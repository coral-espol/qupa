#!/usr/bin/env python3
"""
target_ranger_node — PC-side conversion of mirror-camera detections to metric targets.

Recomputes range and bearing from the raw centroids (cx, cy) using the model
fitted by calibrate_range, so calibration never requires touching the robot.

Subscribes:
  camera/detections   qupa_msgs/DetectionArray   (from the robot)

Publishes:
  camera/targets      qupa_msgs/TargetArray      (base_link frame, metres / rad)
  camera/targets_viz  visualization_msgs/MarkerArray
"""

import rclpy
from rclpy.node import Node
from qupa_msgs.msg import DetectionArray, Target, TargetArray
from visualization_msgs.msg import Marker, MarkerArray

from qupa_desktop.vision_geometry import MirrorModel

MARKER_RGB = {'BLUE': (0.1, 0.3, 1.0), 'GREEN': (0.1, 0.9, 0.2), 'RED': (1.0, 0.1, 0.1)}


class TargetRangerNode(Node):

    def __init__(self):
        super().__init__('target_ranger')

        self.declare_parameter('center_x',         302.0)
        self.declare_parameter('center_y',         254.0)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('angle_direction',  1)
        self.declare_parameter('range_model',      'poly')
        self.declare_parameter('range_coeffs',     [0.0, 0.0])
        self.declare_parameter('range_min_px',     0.0)
        self.declare_parameter('range_max_px',     1000.0)
        self.declare_parameter('base_frame',       '')

        p = {n: self.get_parameter(n).value for n in MirrorModel.PARAM_NAMES}
        self._model = MirrorModel.from_params(p)

        ns = self.get_namespace().strip('/')
        self._frame = self.get_parameter('base_frame').value or \
            (f'{ns}/base_link' if ns else 'base_link')

        if not any(p['range_coeffs']):
            self.get_logger().warn(
                'range_coeffs vacíos — corre calibrate_range y carga vision_<ns>.yaml')

        self._pub = self.create_publisher(TargetArray, 'camera/targets', 10)
        self._viz = self.create_publisher(MarkerArray, 'camera/targets_viz', 10)
        self.create_subscription(DetectionArray, 'camera/detections', self._cb, 10)
        self.get_logger().info(f'target_ranger listo — frame {self._frame}, modelo {p}')

    def _cb(self, msg):
        out = TargetArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, det in enumerate(msg.targets):
            d, th, x, y, r, ok = self._model.project(det.cx, det.cy)
            t = Target()
            t.color       = det.color
            t.distance_m  = float(d)
            t.angle_rad   = float(th)
            t.x           = float(x)
            t.y           = float(y)
            t.distance_px = float(r)
            t.area        = det.area
            t.in_range    = bool(ok)
            out.targets.append(t)

            m = Marker()
            m.header = out.header
            m.ns, m.id = 'targets', i
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = t.x, t.y, 0.03
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.05
            m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b = MARKER_RGB.get(det.color, (1.0, 1.0, 1.0))
            m.color.a = 0.9 if t.in_range else 0.3
            markers.markers.append(m)

        self._pub.publish(out)
        self._viz.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TargetRangerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
