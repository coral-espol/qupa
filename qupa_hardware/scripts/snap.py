#!/usr/bin/env python3
"""
snap.py — Captura un frame con la misma configuración que usa camera_node.

Guarda la imagen en ~/snaps/snap_YYYYMMDD_HHMMSS.jpg y termina.

Uso:
  python3 ~/qupa_ws/src/qupa/qupa_hardware/scripts/snap.py
  python3 ~/qupa_ws/src/qupa/qupa_hardware/scripts/snap.py --yaml ~/qupa_ws/src/qupa/qupa_hardware/config/camera.yaml
"""

import argparse
import os
import time
import yaml
import cv2

from datetime import datetime
from picamera2 import Picamera2
from libcamera import Transform


def load_params(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('camera_node', {}).get('ros__parameters', {})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--yaml',
        default=os.path.expanduser(
            '~/qupa_ws/src/qupa/qupa_hardware/config/camera.yaml'
        ),
        help='Ruta al archivo camera.yaml'
    )
    args = parser.parse_args()

    params   = load_params(args.yaml)
    W        = params.get('image_width',  640)
    H        = params.get('image_height', 480)
    vflip    = params.get('vflip',        True)
    warmup_s = params.get('warmup_s',     2.0)
    quality  = params.get('jpeg_quality', 80)

    # Carpeta de destino
    snap_dir = os.path.expanduser('~/snaps')
    os.makedirs(snap_dir, exist_ok=True)

    # Abrir cámara con la misma configuración que camera_node
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={'size': (W, H), 'format': 'RGB888'},
        transform=Transform(hflip=0, vflip=1 if vflip else 0),
    )
    cam.configure(cfg)
    cam.start()

    print(f'Calentando cámara ({warmup_s}s) ...')
    time.sleep(warmup_s)
    cam.set_controls({'AwbEnable': True, 'AeEnable': True})
    time.sleep(0.5)  # deja estabilizar AWB/AE

    frame = cam.capture_array('main')  # BGR
    cam.stop()

    # Guardar
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path  = os.path.join(snap_dir, f'snap_{timestamp}.jpg')
    cv2.imwrite(out_path, frame, [cv2.IMWRITE_JPEG_QUALITY, quality])

    print(f'Guardado: {out_path}  ({W}x{H})')


if __name__ == '__main__':
    main()
