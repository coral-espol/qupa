#!/usr/bin/env python3
"""
shape_recognition — classify a closed 2-D trajectory as a regular polygon.

Pipeline:
  1. resample the closed path to N points equally spaced along its perimeter
  2. centre on the centroid and scale to unit mean radius
  3. for each candidate k-gon, rotate a unit template over one symmetry
     period and keep the rotation with the lowest symmetric chamfer distance
  4. pick the k with the lowest distance; reject if too high or ambiguous

Chamfer (nearest-point) distance needs no start-point or direction matching,
which makes it robust to where the observed robot started its figure.
"""

import math

import numpy as np

SHAPE_NAMES = {3: 'TRIANGLE', 4: 'SQUARE', 5: 'PENTAGON'}


# ── Geometry helpers ──────────────────────────────────────────────────────────

def resample_closed(xy, n=64):
    """Resample a closed polyline (last point joined to first) to n points."""
    p = np.asarray(xy, dtype=float)
    p = np.vstack([p, p[:1]])
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        raise ValueError('degenerate path (zero length)')
    t = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])])


def smooth_closed(pts, window):
    """Circular moving average over an already-resampled closed path."""
    if window <= 1:
        return pts
    kern = np.ones(window) / window
    pad = window // 2
    ext = np.vstack([pts[-pad:], pts, pts[:window - pad - 1]])
    return np.column_stack([np.convolve(ext[:, i], kern, mode='valid') for i in (0, 1)])


def polygon_vertices(k, circumradius=1.0, rotation=0.0, center=(0.0, 0.0)):
    """Vertices of a regular k-gon, first vertex at angle `rotation`."""
    a = rotation + 2 * math.pi * np.arange(k) / k
    return np.column_stack([center[0] + circumradius * np.cos(a),
                            center[1] + circumradius * np.sin(a)])


def signed_area(xy):
    x, y = np.asarray(xy, dtype=float).T
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def _chamfer(a, b):
    d = np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1])
    return 0.5 * (d.min(axis=1).mean() + d.min(axis=0).mean())


# ── Classifier ────────────────────────────────────────────────────────────────

class PolygonClassifier:

    def __init__(self, sides=(3, 4, 5), n_points=64, rot_step_deg=2.0,
                 smooth_window=1, max_distance=0.25, min_margin=0.0):
        self.sides = tuple(sides)
        self.n = n_points
        self.smooth = smooth_window
        self.max_distance = max_distance
        self.min_margin = min_margin
        # Unit-mean-radius templates, one per (k, rotation)
        self._templates = {}
        for k in self.sides:
            rots = np.radians(np.arange(0.0, 360.0 / k, rot_step_deg))
            base = resample_closed(polygon_vertices(k), self.n)
            mean_r = np.hypot(*base.T).mean()       # circumradius → mean radius ratio
            # Templates get the same smoothing as observations so corners match
            self._templates[k] = (rots, mean_r, [
                smooth_closed(resample_closed(polygon_vertices(k, rotation=r), self.n),
                              self.smooth) / mean_r
                for r in rots])

    def classify(self, xy):
        """Return dict with shape, sides, score, center, circumradius, side_m,
        rotation (first-vertex angle), direction (+1 CCW / -1 CW), vertices,
        and per-k distances."""
        pts = smooth_closed(resample_closed(xy, self.n), self.smooth)
        center = pts.mean(axis=0)
        rel = pts - center
        scale = np.hypot(*rel.T).mean()
        if scale <= 1e-6:
            raise ValueError('degenerate path (no extent)')
        norm = rel / scale

        results = {}
        for k, (rots, mean_r, temps) in self._templates.items():
            dists = [_chamfer(norm, t) for t in temps]
            i = int(np.argmin(dists))
            results[k] = (dists[i], rots[i], scale / mean_r)

        ranked = sorted(results.items(), key=lambda kv: kv[1][0])
        k_best, (d_best, rot, circumradius) = ranked[0]
        margin = ranked[1][1][0] - d_best if len(ranked) > 1 else float('inf')
        ok = d_best <= self.max_distance and margin >= self.min_margin

        return {
            'shape': SHAPE_NAMES.get(k_best, f'{k_best}-GON') if ok else 'UNKNOWN',
            'sides': k_best if ok else 0,
            'best_sides': k_best,
            'score': float(d_best),
            'margin': float(margin),
            'center': (float(center[0]), float(center[1])),
            'circumradius': float(circumradius),
            'side_m': float(2 * circumradius * math.sin(math.pi / k_best)),
            'rotation': float(rot),
            'direction': 1 if signed_area(xy) >= 0 else -1,
            'vertices': polygon_vertices(k_best, circumradius, rot, center),
            'distances': {k: float(v[0]) for k, v in results.items()},
        }
