#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
led_node — ROS2 Jazzy node for the APA102 LED strip.

Subscribes to LED commands and drives the strip.

Subscribes:
  leds/command   std_msgs/String   JSON commands:
    {"mode": "set_all",     "rgb": [r, g, b]}
    {"mode": "set_segment", "rgb": [r, g, b], "from": 0, "to": 7}
    {"mode": "clear"}

Segments (from leds.yaml):
  seg0: 0–7     rear
  seg1: 8–16    front / camera feedback
  seg2: 17–23   side
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

try:
    from apa102_pi.driver import apa102
    _HW = True
except ImportError:
    _HW = False


class LEDNode(Node):

    def __init__(self):
        super().__init__('led_node')

        # ── Parameters ──
        self.declare_parameter('led_count',        24)
        self.declare_parameter('global_brightness', 10)

        # Segment definitions
        self.declare_parameter('seg0_from',  0)
        self.declare_parameter('seg0_to',    7)
        self.declare_parameter('seg1_from',  8)
        self.declare_parameter('seg1_to',   16)
        self.declare_parameter('seg2_from', 17)
        self.declare_parameter('seg2_to',   23)

        count      = self.get_parameter('led_count').value
        brightness = self.get_parameter('global_brightness').value

        self._count = count
        self._segments = {
            'seg0': (self.get_parameter('seg0_from').value,
                     self.get_parameter('seg0_to').value),
            'seg1': (self.get_parameter('seg1_from').value,
                     self.get_parameter('seg1_to').value),
            'seg2': (self.get_parameter('seg2_from').value,
                     self.get_parameter('seg2_to').value),
        }

        # ── Hardware ──
        self._strip = None
        if not _HW:
            self.get_logger().error('apa102_pi not found — cannot init LED strip.')
            return

        try:
            self._strip = apa102.APA102(num_led=count,
                                        global_brightness=brightness)
            self._clear_all()
            self.get_logger().info(f'APA102 strip initialized — {count} LEDs.')
        except Exception as e:
            self.get_logger().error(f'LED strip init failed: {e}')
            return

        # ── Subscriber ──
        self._sub = self.create_subscription(
            String, 'leds/command', self._cmd_cb, 10
        )
        self.get_logger().info('LED node ready — listening on leds/command')

    # ── LED helpers ──────────────────────────────────────────────────────────

    def _clamp(self, i):
        return max(0, min(self._count - 1, i))

    def _set_range(self, start, end, rgb):
        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
        start, end = self._clamp(start), self._clamp(end)
        if start > end:
            start, end = end, start
        for i in range(start, end + 1):
            self._strip.set_pixel(i, r, g, b)
        self._strip.show()

    def _set_all(self, rgb):
        self._set_range(0, self._count - 1, rgb)

    def _clear_all(self):
        for i in range(self._count):
            self._strip.set_pixel(i, 0, 0, 0)
        self._strip.show()

    # ── Subscriber callback ──────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        if self._strip is None:
            return
        try:
            data = json.loads(msg.data)
            mode = data.get('mode', 'set_all')
            rgb  = data.get('rgb', [0, 0, 0])

            if mode == 'set_all':
                self._set_all(rgb)

            elif mode == 'set_segment':
                start = data.get('from', 0)
                end   = data.get('to',   0)
                self._set_range(start, end, rgb)

            elif mode == 'set_named_segment':
                name  = data.get('segment', 'seg0')
                start, end = self._segments.get(name, (0, 0))
                self._set_range(start, end, rgb)

            elif mode == 'clear':
                self._clear_all()

        except Exception as e:
            self.get_logger().error(f'LED command error: {e}')

    def destroy_node(self):
        if self._strip is not None:
            self._clear_all()
            self._strip.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
