#!/usr/bin/env python3
"""
calibrate_mask.py — Herramienta visual para calibrar la máscara de la cámara.

Pasos:
  1. Click en el centro del espejo   → genera inner_offset_x / y
  2. Click en 4 esquinas del pilar 1 → genera pole_line1/2
  3. Click en 4 esquinas del pilar 2 → genera pole_line3/4
  Al final imprime los valores listos para pegar en camera.yaml.

Controles:
  Click izquierdo — agregar punto
  Z               — deshacer último punto
  R               — reiniciar todo
  Q               — salir sin guardar

Uso:
  python3 tools/calibrate_mask.py foto.jpg
"""

import sys
import cv2
import numpy as np

W, H = 640, 480

# ── Estado global ──────────────────────────────────────────────────────────────
ring_center = None   # (x, y) click en el centro del espejo
zones       = [[], []]  # zones[0] = pilar 1 (4 puntos), zones[1] = pilar 2
phase       = 'ring'    # 'ring' → 'zone0' → 'zone1' → 'done'
img_orig    = None

COLORS = [(0, 165, 255), (0, 200, 255)]  # naranja, amarillo

PHASE_LABELS = {
    'ring':  'Paso 1/3 — Click en el CENTRO DEL ESPEJO',
    'zone0': 'Paso 2/3 — Pilar 1: click en las 4 esquinas del area a excluir',
    'zone1': 'Paso 3/3 — Pilar 2: click en las 4 esquinas del area a excluir',
    'done':  'Listo — presiona cualquier tecla para ver el YAML',
}

CORNER_LABELS = ['esquina 1', 'esquina 2', 'esquina 3', 'esquina 4']


# ── Dibujo ─────────────────────────────────────────────────────────────────────

def draw_state(base):
    out = base.copy()

    # Centro del espejo
    if ring_center:
        cx_img, cy_img = W // 2, H // 2
        ox = ring_center[0] - cx_img
        oy = ring_center[1] - cy_img
        cv2.drawMarker(out, ring_center, (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.putText(out, f'offset ({ox:+d}, {oy:+d})',
                    (ring_center[0] + 8, ring_center[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # Zonas de exclusión
    for i, zone in enumerate(zones):
        col = COLORS[i]
        for j, pt in enumerate(zone):
            cv2.circle(out, pt, 6, col, -1)
            cv2.putText(out, str(j + 1), (pt[0] + 7, pt[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
        if len(zone) == 4:
            # Dibujar polígono cerrado
            pts = np.array([zone[0], zone[1], zone[2], zone[3]], dtype=np.int32)
            overlay = out.copy()
            cv2.fillPoly(overlay, [pts], col)
            cv2.addWeighted(overlay, 0.3, out, 0.7, 0, out)
            cv2.polylines(out, [pts[[0, 1, 2, 3, 0]]], True, col, 2)
            label = f'Pilar {i + 1}'
            cx_z = int(np.mean([p[0] for p in zone]))
            cy_z = int(np.mean([p[1] for p in zone]))
            cv2.putText(out, label, (cx_z - 20, cy_z),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    # Instrucción activa
    label = PHASE_LABELS.get(phase, '')
    if phase in ('zone0', 'zone1'):
        z = int(phase[-1])
        n = len(zones[z])
        if n < 4:
            label += f'  →  {CORNER_LABELS[n]} ({n + 1}/4)'
    cv2.rectangle(out, (0, H - 28), (W, H), (30, 30, 30), -1)
    cv2.putText(out, label, (6, H - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

    # Atajo Z
    cv2.putText(out, 'Z=deshacer  R=reiniciar  Q=salir',
                (W - 220, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

    return out


# ── Mouse callback ─────────────────────────────────────────────────────────────

def mouse_cb(event, x, y, flags, param):
    global phase, ring_center

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    if phase == 'ring':
        ring_center = (x, y)
        phase = 'zone0'

    elif phase == 'zone0':
        zones[0].append((x, y))
        if len(zones[0]) == 4:
            phase = 'zone1'

    elif phase == 'zone1':
        zones[1].append((x, y))
        if len(zones[1]) == 4:
            phase = 'done'

    cv2.imshow('Calibracion', draw_state(img_orig))


# ── YAML output ────────────────────────────────────────────────────────────────

def print_yaml():
    print('\n' + '─' * 50)
    print('# Pega estos valores en camera.yaml (sección camera_node)')
    print('─' * 50)

    if ring_center:
        ox = ring_center[0] - W // 2
        oy = ring_center[1] - H // 2
        print(f'\n    inner_offset_x:  {ox}')
        print(f'    inner_offset_y:  {oy}')
        print(f'    outer_offset_x:  {ox}')
        print(f'    outer_offset_y:  {oy}')

    for i, zone in enumerate(zones):
        if len(zone) != 4:
            continue
        base = i * 2 + 1   # 1 para pilar 1, 3 para pilar 2
        # Ordenar: top (menor y primero) y bottom (mayor y)
        sorted_pts = sorted(zone, key=lambda p: p[1])
        top    = sorted(sorted_pts[:2], key=lambda p: p[0])  # izq→der
        bottom = sorted(sorted_pts[2:], key=lambda p: p[0])  # izq→der
        p1, p2 = top
        p3, p4 = bottom
        print(f'\n    pole_line{base}_p1: [{p1[0]}, {p1[1]}]')
        print(f'    pole_line{base}_p2: [{p2[0]}, {p2[1]}]')
        print(f'    pole_line{base + 1}_p1: [{p3[0]}, {p3[1]}]')
        print(f'    pole_line{base + 1}_p2: [{p4[0]}, {p4[1]}]')

    print('\n' + '─' * 50)


# ── Main ───────────────────────────────────────────────────────────────────────

def reset():
    global ring_center, zones, phase
    ring_center = None
    zones       = [[], []]
    phase       = 'ring'


def main():
    global img_orig, phase, ring_center

    if len(sys.argv) < 2:
        print('Uso: python3 tools/calibrate_mask.py <foto.jpg>')
        sys.exit(1)

    img = cv2.imread(sys.argv[1])
    if img is None:
        print(f'Error: no se pudo leer {sys.argv[1]}')
        sys.exit(1)

    img_orig = cv2.resize(img, (W, H))

    cv2.namedWindow('Calibracion')
    cv2.imshow('Calibracion', draw_state(img_orig))
    cv2.waitKey(1)
    cv2.setMouseCallback('Calibracion', mouse_cb)

    while True:
        key = cv2.waitKey(20) & 0xFF

        if key == ord('q'):
            break

        elif key == ord('r'):
            reset()
            cv2.imshow('Calibracion', draw_state(img_orig))

        elif key == ord('z'):
            # Deshacer último punto
            if phase == 'done':
                zones[1].pop()
                phase = 'zone1'
            elif phase == 'zone1':
                if zones[1]:
                    zones[1].pop()
                else:
                    zones[0].pop()
                    phase = 'zone0'
            elif phase == 'zone0':
                if zones[0]:
                    zones[0].pop()
                else:
                    ring_center = None
                    phase = 'ring'
            cv2.imshow('Calibracion', draw_state(img_orig))

        elif phase == 'done':
            print_yaml()
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
