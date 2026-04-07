#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_calibration_node — ROS2 Jazzy node for live mask/HSV tuning in RViz.

Completely independent node: opens its own Picamera2 instance, applies the
ring mask, detects colour blobs, draws the calibration overlay, and publishes
camera/image_calibration/compressed (JPEG).

No subscription to camera/image_raw — does NOT depend on camera_node.

Publishes:
  camera/image_calibration/compressed   sensor_msgs/CompressedImage

Tune parameters live:
    ros2 param set /qupa_3A/camera_calibration_node inner_radius_px 70
    ros2 param set /qupa_3A/camera_calibration_node color_blue_lower "[100,90,70]"
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

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


# ── Node ──────────────────────────────────────────────────────────────────────

class CameraCalibrationNode(Node):

    def __init__(self):
        super().__init__('camera_calibration_node')

        self.declare_parameter('image_width',    640)
        self.declare_parameter('image_height',   480)
        self.declare_parameter('publish_hz',     PUBLISH_HZ_DEFAULT)
        self.declare_parameter('warmup_s',       2.0)
        self.declare_parameter('vflip',          True)
        self.declare_parameter('jpeg_quality',   80)
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

        W          = self.get_parameter('image_width').value
        H          = self.get_parameter('image_height').value
        publish_hz = self.get_parameter('publish_hz').value
        warmup_s   = self.get_parameter('warmup_s').value
        self._vflip   = self.get_parameter('vflip').value
        self._quality = self.get_parameter('jpeg_quality').value
        self._W, self._H = W, H

        self._rebuild_mask()
        self.add_on_set_parameters_callback(self._on_params)

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

        self._pub = self.create_publisher(
            CompressedImage, 'camera/image_calibration/compressed', 10
        )
        self._timer = self.create_timer(1.0 / publish_hz, self._timer_cb)
        self.get_logger().info(
            f'Calibration node ready — {W}×{H} @ {publish_hz:.0f} Hz\n'
            '  Subscribe to camera/image_calibration/compressed in RViz'
        )

    # ── Parameter callback ────────────────────────────────────────────────────

    def _on_params(self, params):
        for p in params:
            if p.name == 'jpeg_quality':
                self._quality = p.value
        self._rebuild_mask()
        return rclpy.node.SetParametersResult(successful=True)

    # ── Mask rebuild ──────────────────────────────────────────────────────────

    def _rebuild_mask(self):
        W  = self.get_parameter('image_width').value
        H  = self.get_parameter('image_height').value
        cx, cy = W / 2.0, H / 2.0

        r_in   = self.get_parameter('inner_radius_px').value
        ox_in  = self.get_parameter('inner_offset_x').value
        oy_in  = self.get_parameter('inner_offset_y').value
        r_out  = self.get_parameter('outer_radius_px').value
        ox_out = self.get_parameter('outer_offset_x').value
        oy_out = self.get_parameter('outer_offset_y').value

        self._ci    = (int(cx + ox_in),  int(cy + oy_in))
        self._co    = (int(cx + ox_out), int(cy + oy_out))
        self._r_in  = r_in
        self._r_out = r_out
        self._ox_in = ox_in
        self._oy_in = oy_in
        self._W, self._H = W, H

        outer = _circular_mask(H, W, (cx + ox_out, cy + oy_out), r_out)
        inner = _circular_mask(H, W, (cx + ox_in,  cy + oy_in),  r_in, invert=True)
        ring  = cv2.bitwise_and(outer, inner)

        def _p(n): return tuple(self.get_parameter(n).value)
        self._lines = [
            (_p('pole_line1_p1'), _p('pole_line1_p2'),
             _p('pole_line2_p1'), _p('pole_line2_p2')),
            (_p('pole_line3_p1'), _p('pole_line3_p2'),
             _p('pole_line4_p1'), _p('pole_line4_p2')),
        ]
        for p1, p2, p3, p4 in self._lines:
            excl = _between_lines_mask(H, W, p1, p2, p3, p4)
            ring = cv2.bitwise_and(ring, cv2.bitwise_not(excl))

        bb = _bbox_from_mask(ring)
        if bb is None:
            self.get_logger().error('Ring mask is empty — check parameters.')
            self._roi = (0, 0, W, H)
            self._ring_roi = ring
        else:
            x0, y0, x1, y1 = bb
            self._roi      = (x0, y0, x1, y1)
            self._ring_roi = ring[y0:y1, x0:x1]

        self._ring_full = ring

        self._colors = {
            'BLUE':  (np.array(self.get_parameter('color_blue_lower').value,  np.uint8),
                      np.array(self.get_parameter('color_blue_upper').value,  np.uint8),
                      (255, 0, 0)),    # BGR blue
            'GREEN': (np.array(self.get_parameter('color_green_lower').value, np.uint8),
                      np.array(self.get_parameter('color_green_upper').value, np.uint8),
                      (0, 255, 0)),    # BGR green
            'RED':   (np.array(self.get_parameter('color_red_lower').value,   np.uint8),
                      np.array(self.get_parameter('color_red_upper').value,   np.uint8),
                      (0, 0, 255)),    # BGR red
        }
        self._min_area = self.get_parameter('min_area').value

    # ── Timer callback ────────────────────────────────────────────────────────

    def _timer_cb(self):
        frame = self._cam.capture_array('main')  # BGR (RGB888 from picamera2)
        now   = self.get_clock().now().to_msg()

        # Apply ring mask — pixels outside ring go black
        out = cv2.bitwise_and(frame, frame, mask=self._ring_full)

        # Draw outer ring (yellow) and inner circle (magenta) + crosshair
        cv2.circle(out, self._co, self._r_out, (0, 255, 255), 2)
        cv2.circle(out, self._ci, self._r_in,  (255, 0, 255), 2)
        cv2.drawMarker(out, self._ci, (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # Draw pole exclusion lines (orange)
        for p1, p2, p3, p4 in self._lines:
            cv2.line(out, p1, p2, (0, 165, 255), 2)
            cv2.line(out, p3, p4, (0, 165, 255), 2)

        # HSV detection on the masked ROI
        rx0, ry0, rx1, ry1 = self._roi
        roi    = out[ry0:ry1, rx0:rx1]
        masked = cv2.bitwise_and(roi, roi, mask=self._ring_roi)
        hsv    = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

        k = np.ones((3, 3), np.uint8)
        ox = (self._W - 1) / 2.0 + self._ox_in
        oy = (self._H - 1) / 2.0 + self._oy_in

        label_y = 20
        for name, (lower, upper, col) in self._colors.items():
            cm = cv2.inRange(hsv, lower, upper)
            cm = cv2.morphologyEx(cm, cv2.MORPH_OPEN,  k)
            cm = cv2.morphologyEx(cm, cv2.MORPH_CLOSE, k)
            cnts, _ = cv2.findContours(cm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                area = cv2.contourArea(cnt)
                if area < self._min_area:
                    continue

                # Bounding rect in full-frame coordinates
                x, y, w, h = cv2.boundingRect(cnt)
                fx, fy = rx0 + x, ry0 + y
                cv2.rectangle(out, (fx, fy), (fx + w, fy + h), col, 2)
                cv2.putText(out, name, (fx, fy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

                # Centroid in full-frame coordinates
                M = cv2.moments(cnt)
                if M['m00'] > 1e-9:
                    cx_b = rx0 + int(M['m10'] / M['m00'])
                    cy_b = ry0 + int(M['m01'] / M['m00'])
                else:
                    cx_b = rx0 + x + w // 2
                    cy_b = ry0 + y + h // 2

                # Orientation arrow from inner-circle center to blob centroid
                vx = float(cx_b - ox)
                vy = float(cy_b - oy)
                theta = math.degrees(math.atan2(vx, -vy))
                dist  = math.hypot(vx, vy)
                cv2.arrowedLine(out,
                                (int(ox), int(oy)),
                                (cx_b, cy_b),
                                col, 2, tipLength=0.2)
                cv2.putText(out,
                            f'{name} D:{dist:.0f}px A:{theta:.1f}deg',
                            (10, label_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                label_y += 20

        # Encode and publish (out is BGR — imencode expects BGR)
        _, buf = cv2.imencode('.jpg', out,
                              [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        pub_msg = CompressedImage()
        pub_msg.header.stamp    = now
        pub_msg.header.frame_id = 'mirror_link'
        pub_msg.format = 'jpeg'
        pub_msg.data   = buf.tobytes()
        self._pub.publish(pub_msg)

    def destroy_node(self):
        if hasattr(self, '_cam'):
            self._cam.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
