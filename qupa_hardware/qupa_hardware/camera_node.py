#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_node — ROS2 Jazzy node for the Raspberry Pi camera (OpenCV V4L2).

Captures at 5 Hz, applies ring mask, runs HSV colour detection, and publishes:
  camera/image_filtered/compressed  — annotated JPEG  (always, ~30–50 KB/frame)
  camera/image_raw                  — raw BGR Image    (only when publish_raw: true,
                                                         for local calibration node use)

640×480 BGR8 uncompressed = ~921 KB/frame → JPEG q=80 ≈ 30–50 KB/frame (~20× less).
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

PUBLISH_HZ = 5.0


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

def _find_best_contour(hsv, lower, upper, min_area):
    mask = cv2.inRange(hsv, lower, upper)
    k = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0
    cnt  = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(cnt))
    return (cnt, area) if area >= min_area else (None, 0)


def _centroid(contour):
    x, y, w, h = cv2.boundingRect(contour)
    M = cv2.moments(contour)
    if M['m00'] > 1e-9:
        return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
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
        self.declare_parameter('image_width',   640)
        self.declare_parameter('image_height',  480)
        self.declare_parameter('warmup_s',      2.0)
        self.declare_parameter('vflip',         True)
        self.declare_parameter('jpeg_quality',  80)
        self.declare_parameter('publish_raw',   False)
        self.declare_parameter('video_device',  0)

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

        W            = self.get_parameter('image_width').value
        H            = self.get_parameter('image_height').value
        warmup_s     = self.get_parameter('warmup_s').value
        self._vflip       = self.get_parameter('vflip').value
        self._quality     = self.get_parameter('jpeg_quality').value
        self._publish_raw = self.get_parameter('publish_raw').value
        self._min_area    = self.get_parameter('min_area').value
        self._W, self._H  = W, H
        self._inner_off_x = self.get_parameter('inner_offset_x').value
        self._inner_off_y = self.get_parameter('inner_offset_y').value

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

        # ── Build ring mask (once at startup) ────────────────────────────────
        cx, cy = W / 2.0, H / 2.0
        r_in   = self.get_parameter('inner_radius_px').value
        ox_in  = self.get_parameter('inner_offset_x').value
        oy_in  = self.get_parameter('inner_offset_y').value
        r_out  = self.get_parameter('outer_radius_px').value
        ox_out = self.get_parameter('outer_offset_x').value
        oy_out = self.get_parameter('outer_offset_y').value

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

        # ── Camera (OpenCV V4L2) ──────────────────────────────────────────────
        device = self.get_parameter('video_device').value
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  W)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open /dev/video{device}.')
            return

        self.get_logger().info(f'Camera warming up for {warmup_s} s …')
        time.sleep(warmup_s)

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_flt = self.create_publisher(
            CompressedImage, 'camera/image_filtered/compressed', 10
        )
        self._pub_raw = self.create_publisher(
            Image, 'camera/image_raw', 10
        ) if self._publish_raw else None

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._timer_cb)
        raw_note = ' + image_raw (local)' if self._publish_raw else ''
        self.get_logger().info(
            f'Camera node ready — {W}×{H} @ {PUBLISH_HZ:.0f} Hz | '
            f'JPEG q={self._quality}{raw_note}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _to_compressed(self, frame, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.format = 'jpeg'
        _, buf = cv2.imencode('.jpg', frame,
                              [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        msg.data = buf.tobytes()
        return msg

    def _to_raw(self, frame, stamp):
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'camera_link'
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = 'bgr8'
        msg.is_bigendian    = 0
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        return msg

    # ── Timer callback ────────────────────────────────────────────────────────

    def _timer_cb(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Frame capture failed — skipping.')
            return

        if self._vflip:
            frame = cv2.flip(frame, 0)

        now = self.get_clock().now().to_msg()

        if self._pub_raw is not None:
            self._pub_raw.publish(self._to_raw(frame, now))

        # Apply ring mask
        x0, y0, x1, y1 = self._roi
        roi_raw    = frame[y0:y1, x0:x1]
        roi_masked = cv2.bitwise_and(roi_raw, roi_raw, mask=self._ring_roi)

        # Detect best colour target
        annotated = frame.copy()
        hsv       = cv2.cvtColor(roi_masked, cv2.COLOR_BGR2HSV)
        best, best_area = None, 0

        for name, (lower, upper, draw_col) in self._colors.items():
            cnt, area = _find_best_contour(hsv, lower, upper, self._min_area)
            if cnt is not None and area > best_area:
                best_area = area
                best = (name, cnt, draw_col)

        if best is not None:
            name, cnt, draw_col = best
            cx_roi, cy_roi = _centroid(cnt)
            cx_g = x0 + cx_roi
            cy_g = y0 + cy_roi
            x0b, y0b, wb, hb = cv2.boundingRect(cnt)
            cv2.rectangle(annotated,
                          (x0 + x0b, y0 + y0b),
                          (x0 + x0b + wb, y0 + y0b + hb), draw_col, 2)
            ox, oy, vx, vy, theta, dist = _orientation(
                cx_g, cy_g, self._W, self._H,
                self._inner_off_x, self._inner_off_y
            )
            cv2.arrowedLine(annotated, (int(ox), int(oy)),
                            (int(ox + vx), int(oy + vy)),
                            (0, 255, 255), 2, tipLength=0.2)
            cv2.putText(annotated,
                        f'{name} D:{dist:.0f}px E:{math.degrees(theta):.1f}deg',
                        (x0 + x0b, y0 + y0b - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, draw_col, 2)

        self._pub_flt.publish(self._to_compressed(annotated, now))

    def destroy_node(self):
        if hasattr(self, '_cap'):
            self._cap.release()
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
