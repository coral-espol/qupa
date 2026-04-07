#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_node — ROS2 Jazzy node for the Raspberry Pi camera.

Captures at publish_hz, applies ring mask, detects ALL colour blobs and
publishes:
  camera/detections          — qupa_msgs/DetectionArray  (always)
  camera/image_filtered/compressed — sensor_msgs/CompressedImage (when publish_image: true)
  camera/image_raw           — sensor_msgs/Image  (when publish_raw: true)
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from qupa_msgs.msg import Detection, DetectionArray

try:
    from picamera2 import Picamera2
    from libcamera import Transform
    _PICAMERA2 = True
except ImportError:
    _PICAMERA2 = False

PUBLISH_HZ_DEFAULT = 3.0


# ── Mask helpers ──────────────────────────────────────────────────────────────

def _circular_mask(h, w, center, radius, invert=False):
    Y, X = np.ogrid[:h, :w]
    dist = np.sqrt((X - center[0]) ** 2 + (Y - center[1]) ** 2)
    mask = (dist <= radius).astype(np.uint8) * 255
    return ~mask if invert else mask


def _between_lines_mask(h, w, p1, p2, p3, p4):
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array([p1, p2, p4, p3], dtype=np.int32)], 255)
    return mask


def _bbox_from_mask(mask):
    ys, xs = np.nonzero(mask)
    if not len(xs):
        return None
    return xs.min(), ys.min(), xs.max() + 1, ys.max() + 1


# ── Detection helpers ─────────────────────────────────────────────────────────

def _find_all_contours(hsv, lower, upper, min_area):
    """Return all contours above min_area for a given HSV range."""
    mask = cv2.inRange(hsv, lower, upper)
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= min_area]


def _centroid(contour):
    M = cv2.moments(contour)
    if M['m00'] > 1e-9:
        return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
    x, y, w, h = cv2.boundingRect(contour)
    return x + w // 2, y + h // 2


def _orientation(cx, cy, W, H, off_x, off_y):
    x0 = (W - 1) / 2.0 + off_x
    y0 = (H - 1) / 2.0 + off_y
    vx, vy = float(cx - x0), float(cy - y0)
    return x0, y0, vx, vy, math.atan2(vx, -vy), math.hypot(vx, vy)


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ── Parameters ──
        self.declare_parameter('image_width',    640)
        self.declare_parameter('image_height',   480)
        self.declare_parameter('publish_hz',     PUBLISH_HZ_DEFAULT)
        self.declare_parameter('warmup_s',       2.0)
        self.declare_parameter('vflip',          True)
        self.declare_parameter('jpeg_quality',   80)
        self.declare_parameter('publish_raw',    False)
        self.declare_parameter('publish_image',  False)  # annotated JPEG
        self.declare_parameter('video_device',   0)

        self.declare_parameter('inner_radius_px', 64)
        self.declare_parameter('inner_offset_x',  -4)
        self.declare_parameter('inner_offset_y',  -38)
        self.declare_parameter('outer_radius_px', 110)
        self.declare_parameter('outer_offset_x',  -4)
        self.declare_parameter('outer_offset_y',  -38)

        self.declare_parameter('pole_line1_p1', [0,   150])
        self.declare_parameter('pole_line1_p2', [250, 190])
        self.declare_parameter('pole_line2_p1', [0,   240])
        self.declare_parameter('pole_line2_p2', [250, 210])
        self.declare_parameter('pole_line3_p1', [100, 0])
        self.declare_parameter('pole_line3_p2', [180, 480])
        self.declare_parameter('pole_line4_p1', [140, 0])
        self.declare_parameter('pole_line4_p2', [220, 480])

        self.declare_parameter('min_area', 50)

        self.declare_parameter('color_blue_lower',  [100,  80,  60])
        self.declare_parameter('color_blue_upper',  [140, 255, 255])
        self.declare_parameter('color_green_lower', [ 35,  80,  80])
        self.declare_parameter('color_green_upper', [ 85, 255, 255])
        self.declare_parameter('color_red_lower',   [  0, 160, 120])
        self.declare_parameter('color_red_upper',   [  8, 255, 255])

        W              = self.get_parameter('image_width').value
        H              = self.get_parameter('image_height').value
        publish_hz     = self.get_parameter('publish_hz').value
        warmup_s       = self.get_parameter('warmup_s').value
        self._vflip         = self.get_parameter('vflip').value
        self._quality       = self.get_parameter('jpeg_quality').value
        self._publish_raw   = self.get_parameter('publish_raw').value
        self._publish_image = self.get_parameter('publish_image').value
        self._min_area      = self.get_parameter('min_area').value
        self._W, self._H    = W, H
        self._inner_off_x   = self.get_parameter('inner_offset_x').value
        self._inner_off_y   = self.get_parameter('inner_offset_y').value

        self._colors = {
            'BLUE':  (np.array(self.get_parameter('color_blue_lower').value,  np.uint8),
                      np.array(self.get_parameter('color_blue_upper').value,  np.uint8),
                      (255, 0, 0)),
            'GREEN': (np.array(self.get_parameter('color_green_lower').value, np.uint8),
                      np.array(self.get_parameter('color_green_upper').value, np.uint8),
                      (0, 255, 0)),
            'RED':   (np.array(self.get_parameter('color_red_lower').value,   np.uint8),
                      np.array(self.get_parameter('color_red_upper').value,   np.uint8),
                      (0, 0, 255)),
        }

        self._build_mask()

        # ── Camera ───────────────────────────────────────────────────────────
        if not _PICAMERA2:
            self.get_logger().error('picamera2 not found — cannot open camera.')
            return

        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={'size': (W, H), 'format': 'RGB888'},
            transform=Transform(hflip=0, vflip=1 if self._vflip else 0),
        )
        self._cam.configure(cfg)
        self._cam.start()

        self.get_logger().info(f'Camera warming up for {warmup_s} s …')
        time.sleep(warmup_s)
        self._cam.set_controls({'AwbEnable': True, 'AeEnable': True})

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_det = self.create_publisher(
            DetectionArray, 'camera/detections', 10
        )
        self._pub_img = self.create_publisher(
            CompressedImage, 'camera/image_filtered/compressed', 10
        ) if self._publish_image else None

        self._pub_raw = self.create_publisher(
            Image, 'camera/image_raw', 10
        ) if self._publish_raw else None

        self._timer = self.create_timer(1.0 / publish_hz, self._timer_cb)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f'Camera node ready — {W}×{H} @ {publish_hz:.0f} Hz | '
            f'image={self._publish_image} raw={self._publish_raw}'
        )

    # ── Mask ─────────────────────────────────────────────────────────────────

    def _build_mask(self):
        W, H = self._W, self._H
        cx, cy = W / 2.0, H / 2.0
        r_in   = self.get_parameter('inner_radius_px').value
        ox_in  = self.get_parameter('inner_offset_x').value
        oy_in  = self.get_parameter('inner_offset_y').value
        r_out  = self.get_parameter('outer_radius_px').value
        ox_out = self.get_parameter('outer_offset_x').value
        oy_out = self.get_parameter('outer_offset_y').value

        self._inner_off_x = ox_in
        self._inner_off_y = oy_in

        outer = _circular_mask(H, W, (cx + ox_out, cy + oy_out), r_out)
        inner = _circular_mask(H, W, (cx + ox_in,  cy + oy_in),  r_in, invert=True)
        ring  = cv2.bitwise_and(outer, inner)

        def _p(n): return tuple(self.get_parameter(n).value)
        for pa, pb, pc, pd in [('pole_line1_p1', 'pole_line1_p2',
                                 'pole_line2_p1', 'pole_line2_p2'),
                                ('pole_line3_p1', 'pole_line3_p2',
                                 'pole_line4_p1', 'pole_line4_p2')]:
            excl = _between_lines_mask(H, W, _p(pa), _p(pb), _p(pc), _p(pd))
            ring = cv2.bitwise_and(ring, cv2.bitwise_not(excl))

        bb = _bbox_from_mask(ring)
        if bb is None:
            self.get_logger().error('Ring mask is empty — check mask parameters.')
            return
        x0, y0, x1, y1 = bb
        self._roi      = (x0, y0, x1, y1)
        self._ring     = ring
        self._ring_roi = ring[y0:y1, x0:x1]

    # ── Parameter callback ────────────────────────────────────────────────────

    def _on_params(self, params):
        rebuild = False
        for p in params:
            if p.name in ('inner_offset_x', 'inner_offset_y',
                          'inner_radius_px', 'outer_radius_px',
                          'outer_offset_x', 'outer_offset_y',
                          'pole_line1_p1', 'pole_line1_p2',
                          'pole_line2_p1', 'pole_line2_p2',
                          'pole_line3_p1', 'pole_line3_p2',
                          'pole_line4_p1', 'pole_line4_p2'):
                rebuild = True
            elif p.name == 'jpeg_quality':
                self._quality = p.value
        if rebuild:
            self._build_mask()
        return rclpy.node.SetParametersResult(successful=True)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _to_compressed(self, frame, stamp):
        msg = CompressedImage()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'mirror_link'
        msg.format = 'jpeg'
        _, buf = cv2.imencode('.jpg', frame,
                              [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        msg.data = buf.tobytes()
        return msg

    def _to_raw(self, frame, stamp):
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'mirror_link'
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = 'rgb8'
        msg.is_bigendian    = 0
        msg.step            = frame.shape[1] * 3
        msg.data            = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).tobytes()
        return msg

    # ── Timer callback ────────────────────────────────────────────────────────

    def _timer_cb(self):
        frame = self._cam.capture_array('main')  # BGR (picamera2 convention)

        now = self.get_clock().now().to_msg()

        if self._pub_raw is not None:
            self._pub_raw.publish(self._to_raw(frame, now))

        # Apply ring mask
        rx0, ry0, rx1, ry1 = self._roi
        roi_raw    = frame[ry0:ry1, rx0:rx1]
        roi_masked = cv2.bitwise_and(roi_raw, roi_raw, mask=self._ring_roi)
        hsv        = cv2.cvtColor(roi_masked, cv2.COLOR_BGR2HSV)

        # Detect ALL blobs per colour
        det_array = DetectionArray()
        det_array.header.stamp    = now
        det_array.header.frame_id = 'mirror_link'

        annotated = frame.copy() if self._pub_img is not None else None

        for name, (lower, upper, draw_col) in self._colors.items():
            contours = _find_all_contours(hsv, lower, upper, self._min_area)
            for cnt in contours:
                cx_roi, cy_roi = _centroid(cnt)
                cx_g = rx0 + cx_roi
                cy_g = ry0 + cy_roi
                _, _, _, _, theta, dist = _orientation(
                    cx_g, cy_g, self._W, self._H,
                    self._inner_off_x, self._inner_off_y
                )
                det = Detection()
                det.color       = name
                det.distance_px = float(dist)
                det.angle_deg   = float(math.degrees(theta))
                det.area        = int(cv2.contourArea(cnt))
                det.cx          = float(cx_g)
                det.cy          = float(cy_g)
                det_array.targets.append(det)

                if annotated is not None:
                    x0b, y0b, wb, hb = cv2.boundingRect(cnt)
                    cv2.rectangle(annotated,
                                  (rx0 + x0b, ry0 + y0b),
                                  (rx0 + x0b + wb, ry0 + y0b + hb),
                                  draw_col, 2)
                    ox = (self._W - 1) / 2.0 + self._inner_off_x
                    oy = (self._H - 1) / 2.0 + self._inner_off_y
                    cv2.arrowedLine(annotated,
                                    (int(ox), int(oy)),
                                    (int(cx_g), int(cy_g)),
                                    (0, 255, 255), 2, tipLength=0.2)
                    cv2.putText(annotated,
                                f'{name} D:{dist:.0f}px E:{math.degrees(theta):.1f}deg',
                                (rx0 + x0b, ry0 + y0b - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_col, 2)

        self._pub_det.publish(det_array)

        if self._pub_img is not None and annotated is not None:
            self._pub_img.publish(self._to_compressed(annotated, now))

    def destroy_node(self):
        if hasattr(self, '_cam'):
            self._cam.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
