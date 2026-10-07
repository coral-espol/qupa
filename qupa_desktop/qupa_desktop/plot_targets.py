#!/usr/bin/env python3
"""
plot_targets.py — plot a target_logger CSV (distance / angle / trajectory).

Usage:
  ros2 run qupa_desktop plot_targets                      # newest CSV in ~/qupa_logs
  ros2 run qupa_desktop plot_targets ~/qupa_logs/targets_qupa_AE_<date>.csv
  ros2 run qupa_desktop plot_targets archivo.csv --color GREEN --show

Writes <csv>.png next to the CSV with:
  - top-down trajectory around the robot (x = front up, y = left to the left)
  - distance vs time and angle vs time, with gaps (target not seen) shaded
  - a short summary (duration, detection rate, gaps, ranges)
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

GOOD_MIN_M, GOOD_MAX_M = 0.20, 0.60   # zone where the distance is accurate
POST_HALF_WIDTH_DEG = 12.0            # approx. blind wedge of each mirror post (±90°)
EXPECTED_HZ = 2.0
MAX_DISTANCE_M = 1.5                  # farther → treated as unknown (e.g. target lifted)


def load(path, color):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if color:
        rows = [r for r in rows if r['color'] == color]
    if not rows:
        raise SystemExit(f'sin datos{" de color " + color if color else ""} en {path}')
    num = {k: np.array([float(r[k]) for r in rows])
           for k in ('t_s', 'distance_m', 'angle_deg', 'x_m', 'y_m', 'in_range')}
    num['in_range'] = num['in_range'].astype(bool)
    # Unknown range (NaN from target_ranger, or absurd values in older logs):
    # keep the bearing, drop distance and position
    bad = ~np.isfinite(num['distance_m']) | (num['distance_m'] > MAX_DISTANCE_M)
    for k in ('distance_m', 'x_m', 'y_m'):
        num[k][bad] = np.nan
    num['unknown'] = bad
    return num


def gaps(t, min_gap):
    """(start, end) of intervals longer than min_gap without detections."""
    d = np.diff(t)
    idx = np.flatnonzero(d > min_gap)
    return [(t[i], t[i + 1]) for i in idx]


def summary(d, gap_list):
    t = d['t_s']
    dur = t[-1] - t[0]
    rate = (len(t) - 1) / dur if dur > 0 else float('nan')
    dist = d['distance_m']
    good = (dist >= GOOD_MIN_M) & (dist <= GOOD_MAX_M)
    return [
        f'duración: {dur:.1f} s   muestras: {len(t)}   '
        f'frecuencia: {rate:.2f} Hz ({100 * min(rate / EXPECTED_HZ, 1):.0f} % de {EXPECTED_HZ:.0f} Hz)',
        f'huecos > 1 s: {len(gap_list)}   tiempo sin ver: {sum(b - a for a, b in gap_list):.1f} s',
        f'distancia: {np.nanmin(dist) * 100:.0f}–{np.nanmax(dist) * 100:.0f} cm   '
        f'en zona precisa ({GOOD_MIN_M * 100:.0f}–{GOOD_MAX_M * 100:.0f} cm): {100 * good.mean():.0f} %   '
        f'fuera de rango calibrado: {100 * (~d["in_range"]).mean():.0f} %   '
        f'distancia desconocida: {d["unknown"].sum()} muestras',
        f'ángulo: {d["angle_deg"].min():.0f}° … {d["angle_deg"].max():.0f}°',
    ]


def plot(d, path, title, show):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Wedge

    t, x, y = d['t_s'], d['x_m'], d['y_m']
    known = ~d['unknown']
    out = ~d['in_range'] & known
    ok = d['in_range'] & known
    gap_list = gaps(t, 1.0)

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.1, 1], height_ratios=[1, 1, 0.45])
    ax_xy = fig.add_subplot(gs[0:2, 0])
    ax_d = fig.add_subplot(gs[0, 1])
    ax_a = fig.add_subplot(gs[1, 1], sharex=ax_d)
    ax_txt = fig.add_subplot(gs[2, :])
    fig.suptitle(title)

    # ── Top-down trajectory: plot (-y, x) so front is up and left is left ──
    lim = max(0.7, float(np.nanmax(np.abs(np.r_[x, y]))) + 0.1)
    for a in (90, -90):     # mirror posts: blind wedges
        th = a + 90         # matplotlib angle for robot angle a in the (-y, x) view
        ax_xy.add_patch(Wedge((0, 0), lim * 1.5, th - POST_HALF_WIDTH_DEG,
                              th + POST_HALF_WIDTH_DEG, color='0.85', zorder=0))
    ax_xy.add_patch(Circle((0, 0), GOOD_MIN_M, fill=False, ls='--', color='0.5'))
    ax_xy.add_patch(Circle((0, 0), GOOD_MAX_M, fill=False, ls='--', color='0.5'))
    ax_xy.annotate('', xy=(0, 0.12), xytext=(0, 0),
                   arrowprops=dict(arrowstyle='->', lw=2, color='k'))
    ax_xy.add_patch(Circle((0, 0), 0.07, color='k', alpha=0.3))
    ax_xy.plot(-y, x, '-', color='0.6', lw=0.8, zorder=1)
    sc = ax_xy.scatter(-y[ok], x[ok], c=t[ok], cmap='viridis', s=18, zorder=2,
                       vmin=t[0], vmax=t[-1])
    if out.any():
        ax_xy.scatter(-y[out], x[out], marker='x', color='tab:red', s=22, zorder=2,
                      label='fuera de rango calibrado')
        ax_xy.legend(loc='lower right', fontsize=8)
    first, last = np.flatnonzero(known)[[0, -1]]
    ax_xy.plot(-y[first], x[first], 'o', mfc='none', mec='tab:green', ms=12, mew=2)
    ax_xy.plot(-y[last], x[last], 's', mfc='none', mec='tab:red', ms=11, mew=2)
    fig.colorbar(sc, ax=ax_xy, label='tiempo (s)', shrink=0.8)
    ax_xy.set_xlim(-lim, lim)
    ax_xy.set_ylim(-lim, lim)
    ax_xy.set_aspect('equal')
    ax_xy.set_xlabel('← izquierda   y (m)   derecha →')
    ax_xy.set_ylabel('← atrás   x (m)   frente →')
    ax_xy.set_title('Trayectoria vista desde arriba  (○ inicio, □ fin; gris = postes)',
                    fontsize=10)
    ax_xy.set_xticks(ax_xy.get_xticks())
    ax_xy.set_xticklabels([f'{-v:.1f}' for v in ax_xy.get_xticks()])
    ax_xy.grid(alpha=0.3)

    # ── Time series ──
    for ax in (ax_d, ax_a):
        for a, b in gap_list:
            ax.axvspan(a, b, color='tab:red', alpha=0.12, lw=0)
        ax.grid(alpha=0.3)
    ax_d.axhspan(GOOD_MIN_M * 100, GOOD_MAX_M * 100, color='tab:green', alpha=0.07)
    ax_d.plot(t, d['distance_m'] * 100, '.-', color='tab:blue', ms=4, lw=0.8)
    for tu in t[d['unknown']]:
        ax_d.axvline(tu, color='tab:purple', alpha=0.4, lw=1)
    if out.any():
        ax_d.plot(t[out], d['distance_m'][out] * 100, 'x', color='tab:red', ms=5)
    ax_d.set_ylabel('distancia (cm)')
    ax_d.set_title('Distancia (verde = zona precisa; rojo = sin detección; '
                   'morado = distancia desconocida)', fontsize=10)
    # Break the line where the angle wraps ±180° (target crossing behind the robot)
    ang = d['angle_deg'].copy()
    wrap = np.flatnonzero(np.abs(np.diff(ang)) > 180) + 1
    ax_a.plot(np.insert(t, wrap, np.nan), np.insert(ang, wrap, np.nan), '.-',
              color='tab:orange', ms=4, lw=0.8)
    ax_a.set_ylabel('ángulo (°)')
    ax_a.set_xlabel('tiempo (s)')
    ax_a.set_ylim(-185, 185)
    ax_a.set_yticks([-180, -90, 0, 90, 180])
    ax_a.set_title('Ángulo (0 = frente, + izquierda)', fontsize=10)

    ax_txt.axis('off')
    ax_txt.text(0.0, 0.95, '\n'.join(summary(d, gap_list)), va='top', family='monospace',
                fontsize=10, transform=ax_txt.transAxes)

    fig.tight_layout()
    png = os.path.splitext(path)[0] + '.png'
    fig.savefig(png, dpi=120)
    print(f'gráfica guardada en {png}')
    if show:
        plt.show()
    return png


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='?', help='CSV de target_logger (default: el más nuevo)')
    ap.add_argument('--color', default='BLUE', help="color a graficar ('' = todos)")
    ap.add_argument('--show', action='store_true', help='abrir además una ventana')
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = ap.parse_args(argv)

    path = args.csv
    if not path:
        files = sorted(glob.glob(os.path.expanduser('~/qupa_logs/targets_*.csv')),
                       key=os.path.getmtime)
        if not files:
            raise SystemExit('no hay CSV en ~/qupa_logs — graba primero con target_logger')
        path = files[-1]
    path = os.path.expanduser(path)

    d = load(path, args.color.upper())
    for line in summary(d, gaps(d['t_s'], 1.0)):
        print(line)
    plot(d, path, os.path.basename(path), args.show)
    return 0


if __name__ == '__main__':
    sys.exit(main())
