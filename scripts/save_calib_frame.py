#!/usr/bin/env python3
"""
save_calib_frame.py — Save one frame of camera/image_calibration/compressed to disk.

Usage (PC, with camera_calibration.launch.py running on the robot):
  python3 save_calib_frame.py --ns qupa_AE --out ~/cartuchera.jpg
"""

import argparse
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='qupa_AE')
    ap.add_argument('--out', default='~/frame.jpg')
    ap.add_argument('--skip', type=int, default=3, help='frames to discard first (AE/AWB settle)')
    args = ap.parse_args()
    out = os.path.expanduser(args.out)

    rclpy.init()
    node = Node('save_calib_frame')
    state = {'n': 0, 'done': False}

    def cb(msg):
        state['n'] += 1
        if state['n'] <= args.skip:
            return
        with open(out, 'wb') as f:
            f.write(bytes(msg.data))
        print(f'guardado {out} ({len(msg.data)} bytes)')
        state['done'] = True

    topic = f'/{args.ns}/camera/image_calibration/compressed'
    node.create_subscription(CompressedImage, topic, cb, 10)
    print(f'esperando {topic} …')
    while rclpy.ok() and not state['done']:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
