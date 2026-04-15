#!/usr/bin/env python3
"""
snap.py — Captura un frame con la misma configuración que usa camera_node.

Guarda la imagen en ~/snaps/snap_YYYYMMDD_HHMMSS.jpg y termina.

Uso:
  # Tomar foto (en el robot)
  python3 snap.py

  # Calibrar máscara sobre una foto (en la PC)
  python3 snap.py --calibrate foto.jpg

  # Apuntar a un yaml distinto
  python3 snap.py --yaml ~/qupa_ws/src/qupa/qupa_hardware/config/camera.yaml
"""

import argparse
import math
import os
import sys
import time
try:
    import tkinter as tk
    from PIL import Image, ImageDraw, ImageTk
except ImportError:
    print('Error: tkinter no disponible. Este modo solo corre en la PC.')


import yaml
import cv2

from datetime import datetime


try:
    from picamera2 import Picamera2
    from libcamera import Transform
    _PICAMERA2 = True
except ImportError:
    _PICAMERA2 = False

W, H = 640, 480


def load_params(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('camera_node', {}).get('ros__parameters', {})


# ── Snap ──────────────────────────────────────────────────────────────────────

def save_image(yaml_path):
    if not _PICAMERA2:
        print('Error: picamera2 no disponible. Este modo solo corre en el robot.')
        sys.exit(1)

    params   = load_params(yaml_path)
    W_       = params.get('image_width',  640)
    H_       = params.get('image_height', 480)
    vflip    = params.get('vflip',        True)
    warmup_s = params.get('warmup_s',     2.0)
    quality  = params.get('jpeg_quality', 80)

    snap_dir = os.path.expanduser('~/snaps')
    os.makedirs(snap_dir, exist_ok=True)

    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={'size': (W_, H_), 'format': 'RGB888'},
        transform=Transform(hflip=0, vflip=1 if vflip else 0),
    )
    cam.configure(cfg)
    cam.start()

    print(f'Calentando camara ({warmup_s}s) ...')
    time.sleep(warmup_s)
    cam.set_controls({'AwbEnable': True, 'AeEnable': True})
    time.sleep(0.5)

    frame = cam.capture_array('main')  # BGR
    cam.stop()

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path  = os.path.join(snap_dir, f'snap_{timestamp}.jpg')
    cv2.imwrite(out_path, frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    print(f'Guardado: {out_path}  ({W_}x{H_})')


# ── Calibrate ─────────────────────────────────────────────────────────────────

def calibrate(src):
    img_pil = Image.open(src).resize((W, H))

    # Estado compartido entre callbacks
    s = {
        'step':    0,       # 0=centro 1=inner 2=outer 3-6=zona1 7-10=zona2
        'center':  None,    # (x, y)
        'r_inner': None,
        'r_outer': None,
        'zone1':   [],      # 4 puntos
        'zone2':   [],      # 4 puntos
        'preview': None,    # posición actual del mouse
    }

    # Labels por punto de zona (en orden de click)
    ZONE_LABELS = [
        'pole_line1_p1  (linea superior — inicio)',
        'pole_line1_p2  (linea superior — fin)',
        'pole_line2_p1  (linea inferior — inicio)',
        'pole_line2_p2  (linea inferior — fin)',
    ]
    ZONE2_LABELS = [
        'pole_line3_p1  (linea superior — inicio)',
        'pole_line3_p2  (linea superior — fin)',
        'pole_line4_p1  (linea inferior — inicio)',
        'pole_line4_p2  (linea inferior — fin)',
    ]

    def step_label():
        step = s['step']
        if step == 0:
            return 'Paso 1/5 — Click en el CENTRO del espejo'
        elif step == 1:
            return 'Paso 2/5 — Click para fijar radio del circulo INTERIOR'
        elif step == 2:
            return 'Paso 3/5 — Click para fijar radio del circulo EXTERIOR'
        elif step <= 6:
            n = len(s['zone1'])
            return f'Paso 4/5 — Pilar 1  [{n+1}/4]:  {ZONE_LABELS[n]}'
        else:
            n = len(s['zone2'])
            return f'Paso 5/5 — Pilar 2  [{n+1}/4]:  {ZONE2_LABELS[n]}'

    def render():
        img = img_pil.copy()
        d   = ImageDraw.Draw(img)

        # Centro
        if s['center']:
            cx, cy = s['center']
            d.line([(cx-14, cy), (cx+14, cy)], fill='white', width=2)
            d.line([(cx, cy-14), (cx, cy+14)], fill='white', width=2)

        # Circulo interior
        if s['r_inner'] is not None and s['center']:
            cx, cy = s['center']
            r = s['r_inner']
            d.ellipse([cx-r, cy-r, cx+r, cy+r], outline='#ff00ff', width=2)

        # Circulo exterior
        if s['r_outer'] is not None and s['center']:
            cx, cy = s['center']
            r = s['r_outer']
            d.ellipse([cx-r, cy-r, cx+r, cy+r], outline='#ffff00', width=2)

        # Preview radio (linea + circulo fantasma)
        if s['preview'] and s['center'] and s['step'] in (1, 2):
            cx, cy = s['center']
            mx, my = s['preview']
            r_p = int(math.hypot(mx-cx, my-cy))
            col = '#ff00ff' if s['step'] == 1 else '#ffff00'
            d.ellipse([cx-r_p, cy-r_p, cx+r_p, cy+r_p], outline=col, width=1)
            d.line([(cx, cy), (mx, my)], fill=col, width=1)
            d.text((mx+6, my-12), f'r={r_p}px', fill=col)

        # Zona 1
        for p in s['zone1']:
            d.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], fill='#ff6600')
        if len(s['zone1']) == 4:
            pts = s['zone1'] + [s['zone1'][0]]
            d.line([(p[0], p[1]) for p in pts], fill='#ff6600', width=2)

        # Zona 2
        for p in s['zone2']:
            d.ellipse([p[0]-4, p[1]-4, p[0]+4, p[1]+4], fill='#00ff88')
        if len(s['zone2']) == 4:
            pts = s['zone2'] + [s['zone2'][0]]
            d.line([(p[0], p[1]) for p in pts], fill='#00ff88', width=2)

        # Preview punto actual del mouse (crosshair)
        if s['preview'] and s['step'] not in (1, 2):
            mx, my = s['preview']
            d.line([(mx-10, my), (mx+10, my)], fill='#aaaaaa', width=1)
            d.line([(mx, my-10), (mx, my+10)], fill='#aaaaaa', width=1)

        # Barra de estado
        d.rectangle([0, H-26, W, H], fill='#1e1e1e')
        d.text((6, H-18), step_label(), fill='#00ffff')
        d.text((W-160, 6), 'R=reiniciar paso  Q=salir', fill='#aaaaaa')

        return ImageTk.PhotoImage(img)

    def on_click(event):
        x, y  = event.x, event.y
        step  = s['step']

        if step == 0:
            s['center'] = (x, y)
            s['step']   = 1
            print(f'[1/5] Centro: ({x}, {y})')

        elif step == 1:
            cx, cy = s['center']
            s['r_inner'] = int(math.hypot(x-cx, y-cy))
            s['step']    = 2
            print(f'[2/5] Radio interior: {s["r_inner"]}px')

        elif step == 2:
            cx, cy = s['center']
            s['r_outer'] = int(math.hypot(x-cx, y-cy))
            s['step']    = 3
            print(f'[3/5] Radio exterior: {s["r_outer"]}px')

        elif step <= 6:
            s['zone1'].append((x, y))
            print(f'[4/5] Zona 1 punto {len(s["zone1"])}/4: ({x}, {y})')
            if len(s['zone1']) == 4:
                s['step'] = 7
                print('[4/5] Zona 1 completa')

        elif step <= 10:
            s['zone2'].append((x, y))
            print(f'[5/5] Zona 2 punto {len(s["zone2"])}/4: ({x}, {y})')
            if len(s['zone2']) == 4:
                s['step'] = 11
                print('[5/5] Zona 2 completa — cerrando y generando YAML...')
                root.after(400, finish)
                return

        refresh()

    def on_move(event):
        s['preview'] = (event.x, event.y)
        refresh()

    def on_key(event):
        if event.char == 'q':
            root.destroy()
        elif event.char == 'r':
            step = s['step']
            if step == 0:
                pass
            elif step == 1:
                s['center'] = None
                s['step']   = 0
            elif step == 2:
                s['r_inner'] = None
                s['step']    = 1
            elif step <= 6:
                s['zone1'] = []
                s['step']  = 3
            elif step <= 10:
                s['zone2'] = []
                s['step']  = 7
            print(f'[R] Reiniciado al paso anterior')
            refresh()

    def refresh():
        photo = render()
        canvas.image = photo
        canvas.itemconfig(img_item, image=photo)

    def finish():
        _print_yaml(s)
        _save_yaml(s)
        root.destroy()

    # Ventana tkinter
    root = tk.Tk()
    root.title('Calibracion de mascara')
    root.resizable(False, False)

    canvas   = tk.Canvas(root, width=W, height=H, cursor='crosshair')
    canvas.pack()

    init_photo = render()
    img_item   = canvas.create_image(0, 0, anchor='nw', image=init_photo)
    canvas.image = init_photo

    canvas.bind('<Button-1>',   on_click)
    canvas.bind('<Motion>',     on_move)
    root.bind('<Key>',          on_key)

    root.mainloop()


def _build_params(s):
    """Construye el dict de parámetros a partir del estado de calibración."""
    cx_img, cy_img = W // 2, H // 2
    p = {}

    if s['center'] and s['r_inner'] is not None:
        ox = s['center'][0] - cx_img
        oy = s['center'][1] - cy_img
        p['inner_radius_px'] = s['r_inner']
        p['inner_offset_x']  = ox
        p['inner_offset_y']  = oy

    if s['center'] and s['r_outer'] is not None:
        ox = s['center'][0] - cx_img
        oy = s['center'][1] - cy_img
        p['outer_radius_px'] = s['r_outer']
        p['outer_offset_x']  = ox
        p['outer_offset_y']  = oy

    for i, zone in enumerate([s['zone1'], s['zone2']]):
        if len(zone) != 4:
            continue
        base       = i * 2 + 1
        p1, p2, p3, p4 = zone   # orden exacto de los clicks
        p[f'pole_line{base}_p1']   = list(p1)
        p[f'pole_line{base}_p2']   = list(p2)
        p[f'pole_line{base+1}_p1'] = list(p3)
        p[f'pole_line{base+1}_p2'] = list(p4)

    return p


def _print_yaml(s):
    params = _build_params(s)
    print('\n' + '─' * 54)
    print('# Pega en camera.yaml  (seccion camera_node:)')
    print('─' * 54)
    for k, v in params.items():
        print(f'    {k}: {v}')
    print('─' * 54)


def _save_yaml(s):
    params   = _build_params(s)
    out_dir  = os.path.expanduser('~/')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'calibration.yaml')

    data = {'camera_node': {'ros__parameters': params}}
    with open(out_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    print(f'\nYAML guardado en: {out_path}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Utilidad de camara QUPA')
    parser.add_argument(
        '--yaml',
        default=os.path.expanduser(
            '~/qupa_ws/src/qupa/qupa_hardware/config/camera.yaml'
        ),
        help='Ruta al archivo camera.yaml'
    )
    parser.add_argument(
        '--calibrate',
        metavar='FOTO',
        help='Calibrar mascara sobre una foto existente'
    )
    args = parser.parse_args()

    if args.calibrate:
        calibrate(args.calibrate)
    else:
        save_image(args.yaml)


if __name__ == '__main__':
    main()
