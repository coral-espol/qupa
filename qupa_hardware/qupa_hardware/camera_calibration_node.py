#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_calibration_node — ROS2 Jazzy node for live mask/HSV tuning in RViz.

Subscribes to camera/image_raw (local Image, requires camera_node publish_raw: true),
draws the ring mask geometry and colour segmentation on top, and publishes
camera/image_calibration/compressed (JPEG) for viewing in RViz on the PC.

Tune parameters live — mask rebuilds immediately, no restart needed:
    ros2 param set /qupa_3A/camera_calibration_node inner_radius_px 70
    ros2 param set /qupa_3A/camera_calibration_node color_blue_lower "[100,90,70]"
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


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


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class CameraCalibrationNode(Node):

    def __init__(self):
        super().__init__('camera_calibration_node')

        # ── Parameters ──
        self.declare_parameter('image_width',  640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('jpeg_quality', 80)

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

        self.declare_parameter('color_blue_lower',  [100,  80,  60])
        self.declare_parameter('color_blue_upper',  [140, 255, 255])
        self.declare_parameter('color_green_lower', [ 35,  80,  80])
        self.declare_parameter('color_green_upper', [ 85, 255, 255])
        self.declare_parameter('color_red_lower',   [  0, 160, 120])
        self.declare_parameter('color_red_upper',   [  8, 255, 255])

        self.declare_parameter('min_area', 50)

        self._quality = self.get_parameter('jpeg_quality').value
        self._rebuild_mask()
        self.add_on_set_parameters_callback(self._on_params)

        self._pub = self.create_publisher(
            CompressedImage, 'camera/image_calibration/compressed', 10
        )
        self._sub = self.create_subscription(
            Image, 'camera/image_raw', self._image_cb, 10
        )

        self.get_logger().info(
            'Calibration node ready — '
            'subscribe to camera/image_calibration/compressed in RViz\n'
            '  Requires camera_node param: publish_raw: true'
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

        self._ci = (int(cx + ox_in),  int(cy + oy_in))
        self._co = (int(cx + ox_out), int(cy + oy_out))
        self._r_in  = r_in
        self._r_out = r_out

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

        self._ring = ring
        self._W, self._H = W, H

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
        self._min_area = self.get_parameter('min_area').value

    # ── Image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        # Decode incoming raw Image manually (avoids cv_bridge dependency)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        out = frame.copy()

        # Semi-transparent green mask overlay
        overlay = np.zeros_like(out)
        overlay[self._ring > 0] = (0, 200, 0)
        out = cv2.addWeighted(out, 1.0, overlay, 0.25, 0)

        # Circles and centre marker
        cv2.circle(out, self._co, self._r_out, (0, 255, 255), 2)
        cv2.circle(out, self._ci, self._r_in,  (255, 0, 255), 2)
        cv2.drawMarker(out, self._ci, (255, 255, 255), cv2.MARKER_CROSS, 20, 2)

        # Pole exclusion lines
        for p1, p2, p3, p4 in self._lines:
            cv2.line(out, p1, p2, (0, 165, 255), 2)
            cv2.line(out, p3, p4, (0, 165, 255), 2)

        # Colour contours from HSV segmentation
        masked = cv2.bitwise_and(frame, frame, mask=self._ring)
        hsv    = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        k      = np.ones((3, 3), np.uint8)
        for name, (lower, upper, col) in self._colors.items():
            cm = cv2.inRange(hsv, lower, upper)
            cm = cv2.morphologyEx(cm, cv2.MORPH_OPEN,  k)
            cm = cv2.morphologyEx(cm, cv2.MORPH_CLOSE, k)
            cnts, _ = cv2.findContours(cm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                if cv2.contourArea(cnt) >= self._min_area:
                    cv2.drawContours(out, [cnt], -1, col, 2)
                    x, y, *_ = cv2.boundingRect(cnt)
                    cv2.putText(out, name, (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

        # Publish as compressed image
        _, buf = cv2.imencode('.jpg', out,
                              [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        pub_msg = CompressedImage()
        pub_msg.header = msg.header
        pub_msg.format = 'jpeg'
        pub_msg.data   = buf.tobytes()
        self._pub.publish(pub_msg)


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
