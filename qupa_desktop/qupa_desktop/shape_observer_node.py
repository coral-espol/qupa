#!/usr/bin/env python3
"""
shape_observer_node — watch another robot trace a figure and recognise it.

The observer robot stays still. Each time the tracked target leaves its rest
position a new trajectory is recorded; as soon as the target comes back to a
point it already visited (a closed loop) that loop is classified as TRIANGLE /
SQUARE / PENTAGON (or UNKNOWN) with shape_recognition.PolygonClassifier. Any
approach path driven before the loop started is discarded.

Subscribes:
  camera/targets      qupa_msgs/TargetArray     (from target_ranger)

Publishes:
  shape/detected      qupa_msgs/Shape           (one message per finished figure)
  shape/viz           visualization_msgs/MarkerArray

Every finished trajectory is also saved as CSV in save_dir (for evaluation).
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from qupa_msgs.msg import Shape, TargetArray
from visualization_msgs.msg import Marker, MarkerArray

from qupa_desktop.shape_recognition import PolygonClassifier

IDLE, RECORDING = 'IDLE', 'RECORDING'


class ShapeObserverNode(Node):

    def __init__(self):
        super().__init__('shape_observer')

        self.declare_parameter('target_color',    'BLUE')
        self.declare_parameter('start_dist_m',    0.05)   # leave rest point → start
        self.declare_parameter('close_dist_m',    0.07)   # back within this of a past point → loop
        self.declare_parameter('min_perimeter_m', 0.35)   # loop must be at least this long
        self.declare_parameter('min_extent_m',    0.12)   # and reach this far from its start
        self.declare_parameter('min_points',      12)
        self.declare_parameter('still_timeout_s', 8.0)    # stopped this long → give up
        self.declare_parameter('lost_timeout_s',  4.0)    # target unseen this long → abort
        self.declare_parameter('max_score',       0.25)
        self.declare_parameter('save_dir',        '~/qupa_shapes')

        g = self.get_parameter
        self._color      = g('target_color').value
        self._start_d    = g('start_dist_m').value
        self._close_d    = g('close_dist_m').value
        self._min_perim  = g('min_perimeter_m').value
        self._min_extent = g('min_extent_m').value
        self._min_pts    = g('min_points').value
        self._still_t    = g('still_timeout_s').value
        self._lost_t     = g('lost_timeout_s').value
        self._save_dir   = os.path.expanduser(g('save_dir').value)
        os.makedirs(self._save_dir, exist_ok=True)

        self._clf = PolygonClassifier(max_distance=g('max_score').value)

        self._state     = IDLE
        self._anchor    = None    # rest position (x, y) while IDLE
        self._path      = []      # [(t, x, y)] while RECORDING
        self._best      = None    # (path, result) of the best unrecognised loop so far
        self._last_seen = None
        self._last_move = None
        self._frame     = 'base_link'

        self._pub = self.create_publisher(Shape, 'shape/detected', 10)
        self._viz = self.create_publisher(MarkerArray, 'shape/viz', 10)
        self.create_subscription(TargetArray, 'camera/targets', self._cb, 10)
        self.create_timer(0.5, self._watchdog)
        self.get_logger().info(
            f'shape_observer listo — siguiendo {self._color}, CSV en {self._save_dir}')

    # ── Tracking ──────────────────────────────────────────────────────────────

    def _pick(self, msg):
        cands = [t for t in msg.targets if t.color == self._color]
        if not cands:
            return None
        ref = self._path[-1][1:] if self._path else self._anchor
        if ref is None:
            t = max(cands, key=lambda c: c.area)
        else:   # nearest to the previous position → keeps the same target
            t = min(cands, key=lambda c: math.hypot(c.x - ref[0], c.y - ref[1]))
        return float(t.x), float(t.y)

    def _cb(self, msg):
        self._frame = msg.header.frame_id or self._frame
        p = self._pick(msg)
        if p is None:
            return
        now = time.monotonic()
        self._last_seen = now

        if self._state == IDLE:
            if self._anchor is None:
                self._anchor = p
            elif math.dist(p, self._anchor) > self._start_d:
                self._state = RECORDING
                self._path = [(now, *self._anchor), (now, *p)]
                self._last_move = now
                self.get_logger().info('movimiento detectado — grabando trayectoria')
            else:   # follow slow drift of the rest point
                self._anchor = (0.8 * self._anchor[0] + 0.2 * p[0],
                                0.8 * self._anchor[1] + 0.2 * p[1])
            return

        # RECORDING
        if math.dist(p, self._path[-1][1:]) > 0.02:
            self._last_move = now
        self._path.append((now, *p))
        self._publish_trail()

        # A loop is accepted only if it is recognised: crossing the approach path
        # also forms a loop, but it will not look like a polygon.
        for i in self._loop_starts(p):
            res = self._classify(self._path[i:])
            if res is None:
                continue
            if self._best is None or res['score'] < self._best[1]['score']:
                self._best = (self._path[i:], res)
            if res['sides'] > 0:
                self._path = self._path[i:]     # drop the approach before the loop
                self._finish(res, 'cerrada')
                return

    def _loop_starts(self, p):
        """Past indices that close a valid loop with p (first of each run, oldest first)."""
        xy = np.array([q[1:] for q in self._path])
        arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        near = np.hypot(xy[:, 0] - p[0], xy[:, 1] - p[1]) <= self._close_d
        ok = near & (arc[-1] - arc >= self._min_perim)
        starts = []
        for i in np.flatnonzero(ok):
            if i > 0 and ok[i - 1]:
                continue
            loop = xy[i:]
            if (len(loop) >= self._min_pts
                    and np.hypot(*(loop - loop[0]).T).max() >= self._min_extent):
                starts.append(int(i))
        return starts

    def _classify(self, path):
        xy = np.array([q[1:] for q in path[:-1]])   # last point ≈ loop start
        try:
            return self._clf.classify(xy)
        except ValueError:
            return None

    def _watchdog(self):
        if self._state != RECORDING:
            return
        now = time.monotonic()
        if self._last_seen is not None and now - self._last_seen > self._lost_t:
            self.get_logger().warn('objetivo perdido — trayectoria descartada')
            self._reset()
        elif self._last_move is not None and now - self._last_move > self._still_t:
            if self._best is not None:      # closed, but never matched a polygon
                self._path, res = self._best
                self._finish(res, 'sin coincidencia')
            else:
                self.get_logger().warn('el objetivo se detuvo sin cerrar la figura — descartada')
                self._reset(save_reason='abierta')

    def _reset(self, save_reason=None):
        if save_reason and len(self._path) >= 3:
            self._save(None, save_reason)
        last = self._path[-1][1:] if self._path else self._anchor
        self._state, self._path, self._anchor = IDLE, [], last
        self._best = None

    # ── Classification ────────────────────────────────────────────────────────

    def _finish(self, res, reason):
        xy = np.array([q[1:] for q in self._path[:-1]])   # same points res was computed on
        msg = Shape()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.shape          = res['shape']
        msg.sides          = res['sides']
        msg.side_m         = res['side_m']
        msg.circumradius_m = res['circumradius']
        msg.center_x, msg.center_y = res['center']
        msg.rotation_rad   = res['rotation']
        msg.direction      = res['direction']
        msg.score          = res['score']
        msg.margin         = res['margin']
        msg.vertices_x     = [float(v) for v in res['vertices'][:, 0]]
        msg.vertices_y     = [float(v) for v in res['vertices'][:, 1]]
        msg.path_x         = [float(v) for v in xy[:, 0]]
        msg.path_y         = [float(v) for v in xy[:, 1]]
        self._pub.publish(msg)
        self._publish_result(msg)

        sentido = 'antihorario' if res['direction'] > 0 else 'horario'
        self.get_logger().info(
            f'FIGURA: {res["shape"]} — lado ≈ {res["side_m"] * 100:.0f} cm, '
            f'sentido {sentido}, score {res["score"]:.3f} (margen {res["margin"]:.3f}), '
            f'{len(xy)} puntos, {self._path[-1][0] - self._path[0][0]:.0f} s')
        self._save(res, reason)
        self._reset()

    def _save(self, res, reason):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        label = res['shape'] if res else 'DESCARTADA'
        path = os.path.join(self._save_dir, f'{stamp}_{label}.csv')
        t0 = self._path[0][0]
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# resultado', label, 'motivo', reason]
                       + ([f'lado_m={res["side_m"]:.3f}', f'score={res["score"]:.3f}',
                           f'margen={res["margin"]:.3f}'] if res else []))
            w.writerow(['t_s', 'x_m', 'y_m'])
            for t, x, y in self._path:
                w.writerow([f'{t - t0:.2f}', f'{x:.4f}', f'{y:.4f}'])
        self.get_logger().info(f'trayectoria guardada en {path}')

    # ── Visualisation ─────────────────────────────────────────────────────────

    def _marker(self, mid, mtype, rgba, width=0.008):
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'shape', mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def _line(self, mid, xs, ys, rgba, closed=False):
        m = self._marker(mid, Marker.LINE_STRIP, rgba)
        pts = [Point(x=float(x), y=float(y), z=0.01) for x, y in zip(xs, ys)]
        if closed and pts:
            pts.append(pts[0])
        m.points = pts
        return m

    def _publish_trail(self):
        xs = [q[1] for q in self._path]
        ys = [q[2] for q in self._path]
        arr = MarkerArray()
        arr.markers.append(self._line(0, xs, ys, (1.0, 0.85, 0.0, 1.0)))
        self._viz.publish(arr)

    def _publish_result(self, msg):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        arr.markers.append(self._line(1, msg.path_x, msg.path_y, (0.6, 0.6, 0.6, 1.0), True))
        ok = msg.sides > 0
        arr.markers.append(self._line(2, msg.vertices_x, msg.vertices_y,
                                      (0.1, 0.9, 0.2, 1.0) if ok else (0.9, 0.2, 0.2, 1.0),
                                      True))
        txt = self._marker(3, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = \
            msg.center_x, msg.center_y, 0.08
        txt.scale.z = 0.05
        txt.text = f'{msg.shape} {msg.side_m * 100:.0f} cm' if ok else 'UNKNOWN'
        arr.markers.append(txt)
        self._viz.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ShapeObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
