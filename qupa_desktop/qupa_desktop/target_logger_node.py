#!/usr/bin/env python3
"""
target_logger_node — log every metric detection (distance, angle, x, y) to CSV.

Subscribes:
  camera/targets      qupa_msgs/TargetArray     (from target_ranger)

Writes one row per detected target to <out_dir>/targets_<ns>_<date>.csv and
prints a short live line per frame. Stop with Ctrl+C; the file is flushed on
every frame, so nothing is lost if the node is killed.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from qupa_msgs.msg import TargetArray

FIELDS = ['t_s', 'stamp_robot', 'color', 'distance_m', 'angle_deg',
          'x_m', 'y_m', 'distance_px', 'area', 'in_range']


class TargetLoggerNode(Node):

    def __init__(self):
        super().__init__('target_logger')

        self.declare_parameter('color',   '')     # '' = log every colour
        self.declare_parameter('out_dir', '~/qupa_logs')
        self.declare_parameter('quiet',   False)

        self._color = self.get_parameter('color').value.upper()
        self._quiet = self.get_parameter('quiet').value
        out_dir = os.path.expanduser(self.get_parameter('out_dir').value)
        os.makedirs(out_dir, exist_ok=True)

        ns = self.get_namespace().strip('/') or 'robot'
        self._path = os.path.join(out_dir, f'targets_{ns}_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        self._file = open(self._path, 'w', newline='')
        self._csv = csv.writer(self._file)
        self._csv.writerow(FIELDS)
        self._t0 = None
        self._rows = 0

        self.create_subscription(TargetArray, 'camera/targets', self._cb, 10)
        self.get_logger().info(
            f'guardando en {self._path} — color: {self._color or "todos"}')

    def _cb(self, msg):
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        shown = []
        for tg in msg.targets:
            if self._color and tg.color != self._color:
                continue
            ang = math.degrees(tg.angle_rad)
            self._csv.writerow([f'{t:.2f}', f'{stamp:.3f}', tg.color,
                                f'{tg.distance_m:.4f}', f'{ang:.2f}',
                                f'{tg.x:.4f}', f'{tg.y:.4f}', f'{tg.distance_px:.1f}',
                                tg.area, int(tg.in_range)])
            self._rows += 1
            flag = '' if tg.in_range else ' (fuera de rango)'
            shown.append(f'{tg.color} {tg.distance_m * 100:5.1f} cm {ang:+6.1f}°{flag}')
        self._file.flush()

        if not self._quiet:
            print(f'[{t:6.1f} s] ' + (' | '.join(shown) if shown else '—'), flush=True)

    def destroy_node(self):
        self._file.close()
        # print, not the logger: after Ctrl+C the ROS context is already shut down
        print(f'{self._rows} filas guardadas en {self._path}', flush=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TargetLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
