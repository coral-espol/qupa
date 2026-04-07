#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
floor_sensor_node — ROS2 Jazzy node for the TCS34725 floor colour sensor.

Reads RGB from the sensor, converts to HSV, classifies against configured
colour ranges, and publishes results at a configurable rate.

Publishes:
  floor/color   std_msgs/String   — JSON: {"label": "CYAN", "hsv": [h, s, v]}
"""

import colorsys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json

try:
    import board
    import adafruit_tcs34725
    _HW = True
except ImportError:
    _HW = False


class ColorClassifier:
    def __init__(self, hsv_colors: dict):
        self._colors = hsv_colors

    def classify(self, h_deg: float, s: float, v: float) -> str:
        for name, spec in self._colors.items():
            s_range  = spec.get('s', [0.0, 1.0])
            v_range  = spec.get('v', [0.0, 1.0])
            if not (s_range[0] <= s <= s_range[1]):
                continue
            if not (v_range[0] <= v <= v_range[1]):
                continue
            for lo, hi in spec.get('h_ranges', []):
                if lo <= h_deg <= hi:
                    return name
        return 'UNKNOWN'


class FloorSensorNode(Node):

    def __init__(self):
        super().__init__('floor_sensor_node')

        # ── Parameters ──
        self.declare_parameter('loop_hz',         5.0)
        self.declare_parameter('integration_time', 0x80)
        self.declare_parameter('gain',             60)

        # HSV colour ranges — flattened as name_h_ranges / name_s / name_v
        # Colors: CYAN, MAGENTA, YELLOW
        self.declare_parameter('color_cyan_h_ranges',    [160, 200])
        self.declare_parameter('color_cyan_s',           [0.40, 1.00])
        self.declare_parameter('color_cyan_v',           [0.20, 1.00])

        self.declare_parameter('color_magenta_h_ranges', [0, 5, 355, 360])
        self.declare_parameter('color_magenta_s',        [0.90, 1.00])
        self.declare_parameter('color_magenta_v',        [0.30, 0.50])

        self.declare_parameter('color_yellow_h_ranges',  [15, 22])
        self.declare_parameter('color_yellow_s',         [0.94, 1.00])
        self.declare_parameter('color_yellow_v',         [0.15, 0.30])

        loop_hz          = self.get_parameter('loop_hz').value
        integration_time = self.get_parameter('integration_time').value
        gain             = self.get_parameter('gain').value

        # Build classifier from params
        self._classifier = self._build_classifier()

        # ── Hardware ──
        self._sensor = None
        if not _HW:
            self.get_logger().error('adafruit_tcs34725 not found — cannot init floor sensor.')
            return

        try:
            self._sensor = self._init_sensor(integration_time, gain)
            self.get_logger().info('TCS34725 floor sensor initialized.')
        except Exception as e:
            self.get_logger().error(f'Floor sensor init failed: {e}')
            return

        self._integration_time = integration_time
        self._gain             = gain
        self._consecutive_errors = 0

        # ── Publisher ──
        self._pub = self.create_publisher(String, 'floor/color', 10)
        self._timer = self.create_timer(1.0 / loop_hz, self._timer_cb)
        self.get_logger().info(f'Floor sensor node ready @ {loop_hz:.1f} Hz')

    def _init_sensor(self, integration_time, gain):
        i2c    = board.I2C()
        sensor = adafruit_tcs34725.TCS34725(i2c)
        sensor.integration_time = integration_time
        sensor.gain             = gain
        return sensor

    def _build_classifier(self):
        def _pair_list(vals):
            return [[vals[i], vals[i+1]] for i in range(0, len(vals), 2)]

        colors = {
            'CYAN': {
                'h_ranges': _pair_list(self.get_parameter('color_cyan_h_ranges').value),
                's': self.get_parameter('color_cyan_s').value,
                'v': self.get_parameter('color_cyan_v').value,
            },
            'MAGENTA': {
                'h_ranges': _pair_list(self.get_parameter('color_magenta_h_ranges').value),
                's': self.get_parameter('color_magenta_s').value,
                'v': self.get_parameter('color_magenta_v').value,
            },
            'YELLOW': {
                'h_ranges': _pair_list(self.get_parameter('color_yellow_h_ranges').value),
                's': self.get_parameter('color_yellow_s').value,
                'v': self.get_parameter('color_yellow_v').value,
            },
        }
        return ColorClassifier(colors)

    def _timer_cb(self):
        if self._sensor is None:
            return

        try:
            r, g, b = self._sensor.color_rgb_bytes
            self._consecutive_errors = 0
        except OSError as e:
            self._consecutive_errors += 1
            self.get_logger().warn(f'I2C read error #{self._consecutive_errors}: {e}')
            if self._consecutive_errors >= 5:
                self.get_logger().warn('Reinitializing sensor...')
                time.sleep(2.0)
                try:
                    self._sensor = self._init_sensor(
                        self._integration_time, self._gain)
                    self._consecutive_errors = 0
                    self.get_logger().info('Sensor reinitialized.')
                except Exception as re:
                    self.get_logger().error(f'Reinit failed: {re}')
            return

        h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
        h_deg   = h * 360.0
        label   = self._classifier.classify(h_deg, s, v)

        msg      = String()
        msg.data = json.dumps({
            'label': label,
            'hsv':   [round(h_deg, 1), round(s, 3), round(v, 3)],
        })
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
