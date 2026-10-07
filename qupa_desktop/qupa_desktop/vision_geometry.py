#!/usr/bin/env python3
"""
vision_geometry — pixel → metric model for the omnidirectional mirror camera.

Model (all parameters come from calibrate_range):

  vx, vy  = cx - center_x, cy - center_y          (image px)
  r_px    = hypot(vx, vy)
  phi     = atan2(vx, -vy)                         (raw image angle, 0 = image up)
  theta   = wrap(angle_direction * (phi - angle_offset))   (REP-103 bearing)
  d_m     = poly(r_px)          if range_model == 'poly'
          = exp(poly(r_px))     if range_model == 'log_poly'

Coefficients are numpy.polyfit order (highest power first).
"""

import math

import numpy as np


def wrap_pi(a):
    """Wrap angle(s) to (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


class MirrorModel:

    def __init__(self, center_x, center_y, angle_offset_deg, angle_direction,
                 range_model, range_coeffs, range_min_px, range_max_px):
        self.cx0 = float(center_x)
        self.cy0 = float(center_y)
        self.phi0 = math.radians(float(angle_offset_deg))
        self.dir = 1.0 if angle_direction >= 0 else -1.0
        if range_model not in ('poly', 'log_poly'):
            raise ValueError(f'unknown range_model {range_model!r}')
        self.range_model = range_model
        self.coeffs = np.asarray(range_coeffs, dtype=float)
        self.r_min = float(range_min_px)
        self.r_max = float(range_max_px)

    # ── Components ────────────────────────────────────────────────────────────

    def polar_px(self, cx, cy):
        """(r_px, phi_raw) from image centroid."""
        vx = np.asarray(cx, dtype=float) - self.cx0
        vy = np.asarray(cy, dtype=float) - self.cy0
        return np.hypot(vx, vy), np.arctan2(vx, -vy)

    def bearing(self, phi):
        return wrap_pi(self.dir * (np.asarray(phi, dtype=float) - self.phi0))

    def distance(self, r_px):
        y = np.polyval(self.coeffs, np.asarray(r_px, dtype=float))
        return np.exp(y) if self.range_model == 'log_poly' else y

    def in_range(self, r_px):
        r = np.asarray(r_px, dtype=float)
        return (r >= self.r_min) & (r <= self.r_max)

    # ── Full pipeline ─────────────────────────────────────────────────────────

    def project(self, cx, cy):
        """Return (distance_m, angle_rad, x_m, y_m, r_px, in_range)."""
        r, phi = self.polar_px(cx, cy)
        th = self.bearing(phi)
        d = self.distance(r)
        return d, th, d * np.cos(th), d * np.sin(th), r, self.in_range(r)

    # ── Serialisation ─────────────────────────────────────────────────────────

    PARAM_NAMES = ('center_x', 'center_y', 'angle_offset_deg', 'angle_direction',
                   'range_model', 'range_coeffs', 'range_min_px', 'range_max_px')

    def to_params(self):
        return {
            'center_x':         round(self.cx0, 2),
            'center_y':         round(self.cy0, 2),
            'angle_offset_deg': round(math.degrees(self.phi0), 3),
            'angle_direction':  int(self.dir),
            'range_model':      self.range_model,
            'range_coeffs':     [float(f'{c:.8g}') for c in self.coeffs],
            'range_min_px':     round(self.r_min, 2),
            'range_max_px':     round(self.r_max, 2),
        }

    @classmethod
    def from_params(cls, p):
        return cls(**{k: p[k] for k in cls.PARAM_NAMES})
