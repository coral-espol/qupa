#!/usr/bin/env python3
"""
calibrate_range.py — Calibra distancia (m) y orientación (rad) de la cámara espejo.

Corre en la PC. Escucha <ns>/camera/detections del robot y guarda puntos de
calibración (distancia y ángulo reales medidos con cinta métrica) en un CSV.
Luego ajusta el modelo de MirrorModel (centro óptico, offset/sentido del ángulo
y curva r_px → d_m) e imprime el YAML para config/vision_<ns>.yaml.

Modos:
  ros2 run qupa_desktop calibrate_range record --ns qupa_3A --csv datos.csv
  ros2 run qupa_desktop calibrate_range fit datos.csv --out vision_qupa_3A.yaml
  ros2 run qupa_desktop calibrate_range eval vision_qupa_3A.yaml validacion.csv

Comandos dentro de 'record':
  p <d_m> <ang_deg>   graba un punto: objetivo a d_m metros del centro del robot,
                      ángulo real en grados (0 = frente, positivo = izquierda/CCW).
                      Escribe UN comando por línea, sin comentarios.
  p! <d_m> <ang_deg>  igual, pero graba aunque el blob no se haya movido
  c <COLOR>           cambia el color seguido (BLUE | GREEN)
  n <frames>          frames promediados por punto (default 9 ≈ 4.5 s a 2 Hz)
  s                   estado de la última detección
  u                   deshace el último punto
  f                   ajusta el modelo con lo grabado hasta ahora
  q                   salir
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import yaml

from qupa_desktop.vision_geometry import MirrorModel, wrap_pi

CSV_FIELDS = ['stamp', 'color', 'd_true_m', 'theta_true_deg',
              'cx', 'cy', 'cx_std', 'cy_std', 'area', 'n_frames']


# ── CSV helpers ───────────────────────────────────────────────────────────────

def load_csv(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in CSV_FIELDS:
            if k != 'color':
                r[k] = float(r[k])
    return rows


def save_csv(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)


# ── Fitting ───────────────────────────────────────────────────────────────────

def fit_circle(x, y):
    """Algebraic (Kåsa) least-squares circle fit → (xc, yc, R)."""
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x ** 2 + y ** 2
    (a0, a1, a2), *_ = np.linalg.lstsq(A, b, rcond=None)
    xc, yc = a0 / 2, a1 / 2
    return xc, yc, math.sqrt(a2 + xc ** 2 + yc ** 2)


def fit_center(rows, min_angles=3, min_span_deg=120):
    """Mirror centre from points at the same distance but different angles.

    Each distance group whose angles span ≥ min_span_deg is fitted with a
    circle; centres are averaged weighted by the number of points.
    """
    groups = {}
    for r in rows:
        groups.setdefault(round(r['d_true_m'], 3), []).append(r)

    centres, weights = [], []
    for d, g in sorted(groups.items()):
        angs = sorted({round(r['theta_true_deg']) for r in g})
        if len(angs) < min_angles:
            continue
        gaps = np.diff(angs + [angs[0] + 360])
        if 360 - gaps.max() < min_span_deg:
            continue
        x = np.array([r['cx'] for r in g])
        y = np.array([r['cy'] for r in g])
        xc, yc, R = fit_circle(x, y)
        res = np.hypot(x - xc, y - yc) - R
        print(f'  círculo d={d:.3f} m: centro=({xc:.1f}, {yc:.1f}) R={R:.1f} px '
              f'residuo RMS={np.sqrt(np.mean(res ** 2)):.2f} px  (n={len(g)})')
        centres.append((xc, yc))
        weights.append(len(g))

    if not centres:
        return None
    c = np.average(np.array(centres), axis=0, weights=weights)
    return float(c[0]), float(c[1])


def fit_angle(phi, theta):
    """Find (offset, direction) with theta = wrap(dir * (phi - offset))."""
    best = None
    for d in (1.0, -1.0):
        # phi - dir*theta = offset  → circular mean
        z = np.exp(1j * (phi - d * theta))
        off = float(np.angle(z.mean()))
        res = wrap_pi(d * (phi - off) - theta)
        rms = float(np.sqrt(np.mean(res ** 2)))
        if best is None or rms < best[2]:
            best = (off, d, rms)
    return best


def fit_range(r, d, max_deg=3):
    """Pick poly / log_poly of degree 1..max_deg by leave-one-out RMSE (m)."""
    n = len(r)
    candidates = []
    for model in ('poly', 'log_poly'):
        y = np.log(d) if model == 'log_poly' else d
        for deg in range(1, max_deg + 1):
            if n < deg + 3:        # need slack for LOO to mean anything
                continue
            errs = []
            for i in range(n):
                m = np.arange(n) != i
                c = np.polyfit(r[m], y[m], deg)
                p = np.polyval(c, r[i])
                errs.append((np.exp(p) if model == 'log_poly' else p) - d[i])
            loo = float(np.sqrt(np.mean(np.square(errs))))
            candidates.append((loo, model, deg, np.polyfit(r, y, deg)))
    if not candidates:
        raise ValueError(f'se necesitan ≥4 distancias distintas (hay {n})')
    candidates.sort(key=lambda t: t[0])
    print('  modelos de distancia (RMSE leave-one-out):')
    for loo, model, deg, _ in candidates:
        print(f'    {model:8s} grado {deg}: {loo * 100:6.2f} cm')
    loo, model, deg, coeffs = candidates[0]
    return model, coeffs, loo


def fit_model(rows, center=None):
    if len(rows) < 5:
        raise ValueError(f'muy pocos puntos ({len(rows)}); graba al menos 5')

    print('\n── Centro óptico ─────────────────────────────')
    if center is None:
        center = fit_center(rows)
    if center is None:
        raise ValueError(
            'no se pudo estimar el centro: graba ≥3 ángulos que cubran ≥120° a una '
            'misma distancia, o pasa --center X Y')
    print(f'  centro = ({center[0]:.1f}, {center[1]:.1f}) px')

    cx = np.array([r['cx'] for r in rows])
    cy = np.array([r['cy'] for r in rows])
    d_true = np.array([r['d_true_m'] for r in rows])
    th_true = np.radians([r['theta_true_deg'] for r in rows])

    r_px = np.hypot(cx - center[0], cy - center[1])
    phi = np.arctan2(cx - center[0], -(cy - center[1]))

    print('\n── Ángulo ────────────────────────────────────')
    off, direction, _ = fit_angle(phi, th_true)
    print(f'  offset = {math.degrees(off):.2f}°, sentido = {int(direction):+d}')

    print('\n── Distancia ─────────────────────────────────')
    # Average repeated distances so LOO is per distance, not per sample
    uniq = np.unique(np.round(d_true, 3))
    r_mean = np.array([r_px[np.isclose(d_true, u, atol=5e-4)].mean() for u in uniq])
    order = np.argsort(r_mean)
    if np.any(np.diff(uniq[order]) <= 0):
        print('  AVISO: la distancia no crece monótonamente con r_px — revisa los datos')
    model, coeffs, loo = fit_range(r_mean, uniq)
    print(f'  elegido: {model} grado {len(coeffs) - 1}')

    mm = MirrorModel(center[0], center[1], math.degrees(off), direction, model, coeffs,
                     r_px.min(), r_px.max())
    return mm, loo


def report(mm, rows, plot_path=None):
    cx = np.array([r['cx'] for r in rows])
    cy = np.array([r['cy'] for r in rows])
    d_true = np.array([r['d_true_m'] for r in rows])
    th_true = np.radians([r['theta_true_deg'] for r in rows])
    d, th, x, y, r_px, _ = mm.project(cx, cy)

    e_d = d - d_true
    e_th = wrap_pi(th - th_true)
    e_xy = np.hypot(x - d_true * np.cos(th_true), y - d_true * np.sin(th_true))

    print(f'\n── Error ({len(rows)} puntos) ─────────────────────')
    print(f'  distancia : RMSE {np.sqrt(np.mean(e_d ** 2)) * 100:.2f} cm, '
          f'máx {np.abs(e_d).max() * 100:.2f} cm')
    print(f'  ángulo    : RMSE {math.degrees(np.sqrt(np.mean(e_th ** 2))):.2f}°, '
          f'máx {math.degrees(np.abs(e_th).max()):.2f}°')
    print(f'  posición  : RMSE {np.sqrt(np.mean(e_xy ** 2)) * 100:.2f} cm')
    print('\n  d_real  ang_real  r_px    d_est   ang_est   err_d(cm) err_ang(°)')
    for i in np.argsort(d_true * 1000 + np.degrees(th_true) / 1000):
        print(f'  {d_true[i]:6.3f}  {math.degrees(th_true[i]):7.1f}  {r_px[i]:6.1f}  '
              f'{d[i]:6.3f}  {math.degrees(th[i]):7.1f}   {e_d[i] * 100:+7.2f}  '
              f'{math.degrees(e_th[i]):+7.2f}')

    if not plot_path:
        return
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('  (matplotlib no disponible — sin gráfica)')
        return

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    rr = np.linspace(mm.r_min, mm.r_max, 200)
    ax[0].plot(r_px, d_true, 'o', label='medido')
    ax[0].plot(rr, mm.distance(rr), '-', label=f'{mm.range_model} grado {len(mm.coeffs) - 1}')
    ax[0].set_xlabel('r (px)')
    ax[0].set_ylabel('distancia (m)')
    ax[0].set_title('Curva r_px → d_m')
    ax[0].legend()
    ax[0].grid(alpha=.3)

    ax[1].plot(np.degrees(th_true), np.degrees(e_th), 'o')
    ax[1].axhline(0, color='k', lw=.8)
    ax[1].set_xlabel('ángulo real (°)')
    ax[1].set_ylabel('error de ángulo (°)')
    ax[1].set_title('Error angular')
    ax[1].grid(alpha=.3)

    ax[2].plot(d_true * np.cos(th_true), d_true * np.sin(th_true), 'o', label='real')
    ax[2].plot(x, y, 'x', label='estimado')
    for i in range(len(x)):
        ax[2].plot([d_true[i] * np.cos(th_true[i]), x[i]],
                   [d_true[i] * np.sin(th_true[i]), y[i]], 'k-', lw=.5)
    ax[2].plot(0, 0, 'k^', ms=10)
    ax[2].set_aspect('equal')
    ax[2].set_xlabel('x base_link (m)')
    ax[2].set_ylabel('y base_link (m)')
    ax[2].set_title('Posición real vs estimada')
    ax[2].legend()
    ax[2].grid(alpha=.3)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    print(f'\n  gráfica guardada en {plot_path}')


def write_yaml(mm, path):
    with open(path, 'w') as f:
        f.write('# Generado por calibrate_range — modelo pixel → métrico de la cámara espejo\n')
        yaml.safe_dump({'/**': {'ros__parameters': mm.to_params()}}, f,
                       sort_keys=False, default_flow_style=None)
    print(f'  YAML guardado en {path}')


def cmd_fit(args):
    rows = load_csv(args.csv)
    try:
        mm, _ = fit_model(rows, tuple(args.center) if args.center else None)
    except ValueError as e:
        print(f'ERROR: {e}')
        return 1
    report(mm, rows, args.plot or os.path.splitext(args.csv)[0] + '.png')
    print('\n── Parámetros ────────────────────────────────')
    print(yaml.safe_dump(mm.to_params(), sort_keys=False, default_flow_style=None))
    if args.out:
        write_yaml(mm, args.out)
    return 0


def cmd_eval(args):
    with open(args.yaml) as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    mm = MirrorModel.from_params(params)
    rows = load_csv(args.csv)
    print(f'Evaluando {args.yaml} sobre {len(rows)} puntos de {args.csv} (no usados en el ajuste)')
    report(mm, rows, args.plot or os.path.splitext(args.csv)[0] + '_eval.png')
    return 0


# ── Recording ─────────────────────────────────────────────────────────────────

def cmd_record(args):
    import rclpy
    from rclpy.node import Node
    from qupa_msgs.msg import DetectionArray

    class Listener(Node):

        def __init__(self):
            super().__init__('calibrate_range')
            self.lock = threading.Lock()
            self.last = None          # (t_wall, DetectionArray)
            self.frames = []          # buffer while recording
            self.recording = False
            topic = f'/{args.ns}/camera/detections' if args.ns else 'camera/detections'
            self.create_subscription(DetectionArray, topic, self.cb, 10)
            self.get_logger().info(f'escuchando {topic}')

        def cb(self, msg):
            with self.lock:
                self.last = (time.time(), msg)
                if self.recording:
                    self.frames.append(msg)

    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = Listener()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    rows = load_csv(args.csv) if os.path.exists(args.csv) else []
    color, n_frames = args.color.upper(), args.frames
    print(__doc__.split('Comandos')[1])
    print(f'CSV: {args.csv} ({len(rows)} puntos previos) — color {color}, {n_frames} frames/punto')

    def biggest(msg):
        c = [t for t in msg.targets if t.color == color]
        return max(c, key=lambda t: t.area) if c else None

    try:
        while True:
            try:
                line = input(f'[{len(rows)} pts | {color}] > ').strip()
            except EOFError:
                break
            if not line:
                continue
            cmd, *rest = line.split()
            cmd = cmd.lower()

            if cmd == 'q':
                break

            elif cmd == 'c' and rest:
                color = rest[0].upper()

            elif cmd == 'n' and rest:
                n_frames = max(1, int(rest[0]))

            elif cmd == 's':
                with node.lock:
                    last = node.last
                if last is None:
                    print('  sin mensajes todavía — ¿corre camera_node en el robot? ¿mismo ROS_DOMAIN_ID?')
                    continue
                age = time.time() - last[0]
                t = biggest(last[1])
                all_c = ', '.join(f'{x.color}(a={x.area})' for x in last[1].targets) or '—'
                print(f'  hace {age:.1f} s: {all_c}')
                if t:
                    print(f'  {color}: cx={t.cx:.1f} cy={t.cy:.1f} r_robot={t.distance_px:.1f}px '
                          f'ang_robot={t.angle_deg:.1f}°')

            elif cmd == 'u':
                if rows:
                    print(f'  eliminado: {rows.pop()}')
                    save_csv(args.csv, rows)

            elif cmd in ('p', 'p!'):
                try:
                    if len(rest) != 2:
                        raise ValueError
                    d_true, ang_true = float(rest[0]), float(rest[1])
                except ValueError:
                    print('  uso: p <d_m> <ang_deg>   (ej: p 0.30 45) — nada más en la línea')
                    continue
                if not (0.0 < d_true <= 3.0 and -180.0 <= ang_true <= 180.0):
                    print('  fuera de rango: 0 < d_m ≤ 3, -180 ≤ ang_deg ≤ 180 — no se grabó')
                    continue
                with node.lock:
                    node.frames, node.recording = [], True
                t0 = time.time()
                timeout = max(10.0, n_frames * 1.5)
                while time.time() - t0 < timeout:
                    with node.lock:
                        if len(node.frames) >= n_frames:
                            break
                    time.sleep(0.05)
                with node.lock:
                    node.recording = False
                    frames = list(node.frames)

                hits = [b for b in (biggest(m) for m in frames) if b is not None]
                if len(hits) < max(3, n_frames // 2):
                    print(f'  FALLO: {color} visto en {len(hits)}/{len(frames)} frames — no se grabó')
                    continue
                xs = np.array([h.cx for h in hits])
                ys = np.array([h.cy for h in hits])

                # Guard: same blob position as an earlier point with a different truth
                # → the target was not moved (or another object is being tracked).
                same = [r for r in rows
                        if math.hypot(r['cx'] - np.median(xs), r['cy'] - np.median(ys)) < 1.0
                        and (abs(r['d_true_m'] - d_true) > 1e-3
                             or abs(r['theta_true_deg'] - ang_true) > 1e-3)]
                if same and cmd != 'p!':
                    r = same[-1]
                    print(f'  NO se grabó: el blob está donde mismo que el punto '
                          f'({r["d_true_m"]} m, {r["theta_true_deg"]}°). ¿Moviste el objetivo? '
                          f'¿Hay otro objeto {color} más grande a la vista? (usa p! para forzar)')
                    continue
                multi = max(sum(t.color == color for t in m.targets) for m in frames)
                row = {
                    'stamp': time.time(), 'color': color,
                    'd_true_m': d_true, 'theta_true_deg': ang_true,
                    'cx': float(np.median(xs)), 'cy': float(np.median(ys)),
                    'cx_std': float(xs.std()), 'cy_std': float(ys.std()),
                    'area': float(np.median([h.area for h in hits])),
                    'n_frames': len(hits),
                }
                rows.append(row)
                save_csv(args.csv, rows)
                warn = '  ← ¡ruidoso!' if max(row['cx_std'], row['cy_std']) > 3 else ''
                if multi > 1:
                    warn += f'  ← ¡{multi} blobs {color} a la vista! se usó el más grande'
                print(f'  OK: cx={row["cx"]:.1f}±{row["cx_std"]:.1f} '
                      f'cy={row["cy"]:.1f}±{row["cy_std"]:.1f} '
                      f'área={row["area"]:.0f} ({len(hits)} frames){warn}')

            elif cmd == 'f':
                try:
                    mm, _ = fit_model(rows)
                    report(mm, rows)
                except ValueError as e:
                    print(f'  {e}')

            else:
                print('  comando no reconocido (p <d_m> <ang_deg> | p! | c | n | s | u | f | q)')
    except KeyboardInterrupt:
        print()
    finally:
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='mode', required=True)

    rec = sub.add_parser('record', help='grabar puntos desde el robot')
    rec.add_argument('--ns', default='qupa_3A', help='namespace del robot')
    rec.add_argument('--csv', default='range_calib.csv')
    rec.add_argument('--color', default='BLUE')
    rec.add_argument('--frames', type=int, default=9)

    fit = sub.add_parser('fit', help='ajustar el modelo desde un CSV')
    fit.add_argument('csv')
    fit.add_argument('--out', help='YAML de salida (p.ej. config/vision_qupa_3A.yaml)')
    fit.add_argument('--plot', help='PNG de salida (default: <csv>.png)')
    fit.add_argument('--center', type=float, nargs=2, metavar=('X', 'Y'),
                     help='fija el centro óptico en vez de estimarlo')

    ev = sub.add_parser('eval', help='validar un YAML ya ajustado con otro CSV')
    ev.add_argument('yaml')
    ev.add_argument('csv')
    ev.add_argument('--plot', help='PNG de salida (default: <csv>_eval.png)')

    # ros2 run passes --ros-args …; strip them
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = ap.parse_args(argv)
    return {'record': cmd_record, 'fit': cmd_fit, 'eval': cmd_eval}[args.mode](args)


if __name__ == '__main__':
    sys.exit(main())
