import math

import numpy as np
import pytest

from qupa_desktop.shape_recognition import PolygonClassifier, polygon_vertices

CLF = PolygonClassifier()


def _traced(k, side, center=(0.45, 0.0), rot=0.3, reverse=False, noise=0.0, seed=0):
    """Points sampled along a k-gon as a robot would trace it at 2 Hz."""
    rng = np.random.default_rng(seed)
    R = side / (2 * math.sin(math.pi / k))
    V = polygon_vertices(k, R, rot, center)
    if reverse:
        V = V[::-1]
    pts = []
    for i in range(k):
        a, b = V[i], V[(i + 1) % k]
        n = max(2, int(np.hypot(*(b - a)) / 0.04))
        pts += [a + (b - a) * j / n for j in range(n)]
    return np.array(pts) + rng.normal(0, noise, (len(pts), 2))


@pytest.mark.parametrize('k,name', [(3, 'TRIANGLE'), (4, 'SQUARE'), (5, 'PENTAGON')])
def test_clean_polygons(k, name):
    res = CLF.classify(_traced(k, 0.30))
    assert res['shape'] == name
    assert res['side_m'] == pytest.approx(0.30, abs=0.02)
    assert res['center'] == pytest.approx((0.45, 0.0), abs=0.01)


@pytest.mark.parametrize('k', [3, 4, 5])
def test_noisy_polygons(k):
    hits = sum(CLF.classify(_traced(k, 0.30, rot=s, noise=0.01, seed=s))['sides'] == k
               for s in range(20))
    assert hits >= 18


def test_direction():
    assert CLF.classify(_traced(4, 0.3))['direction'] == 1
    assert CLF.classify(_traced(4, 0.3, reverse=True))['direction'] == -1


def test_rejects_back_and_forth_line():
    x = np.r_[np.linspace(-0.15, 0.15, 20), np.linspace(0.15, -0.15, 20)]
    res = CLF.classify(np.column_stack([0.45 + x, np.zeros_like(x)]))
    assert res['shape'] == 'UNKNOWN'
