#!/usr/bin/env python3
import time
import os
from datetime import datetime
import math
import logging
import numpy as np
import cv2
import yaml  
from picamera2 import Picamera2
from libcamera import Transform 
from apa102_pi.driver import apa102

# ================= CONFIGURACIÓN DE RUTAS =================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CAM_CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "camera.yaml")
LED_CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "leds.yaml")

def load_config(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"No se encontró el archivo de configuración en {path}")   
    with open(path, 'r') as f:
        return yaml.safe_load(f)

# Cargar configuraciones por separado
cfg_cam = load_config(CAM_CONFIG_PATH)
cfg_led = load_config(LED_CONFIG_PATH)

# ================= VARIABLES CÁMARA =================
# Nota: Ahora usamos 'image' como clave raíz según tu nuevo YAML
IMAGE_SIZE = tuple(cfg_cam['image']['image_size']) 
WARMUP_S = cfg_cam['image']['warmup_s']
LOCK_AWB_AE = cfg_cam['image']['lock_awb_ae']

R_INNER = cfg_cam['masks']['inner']['radius_px']
OFF_INNER = tuple(cfg_cam['masks']['inner']['offset'])
R_OUTER = cfg_cam['masks']['outer']['radius_px']
OFF_OUTER = tuple(cfg_cam['masks']['outer']['offset'])

POLE_LINES = cfg_cam['masks'].get('pole_lines', None)

MIN_AREA = cfg_cam['detection']['min_area']
SAVE_EVERY_S = cfg_cam['detection']['save_every_s']
SAVE_DIR = cfg_cam['detection']['save_dir']
if not os.path.isabs(SAVE_DIR):
    SAVE_DIR = os.path.join(PROJECT_ROOT, SAVE_DIR)

COLORS_CONFIG = {}
for name, data in cfg_cam['colors'].items():
    COLORS_CONFIG[name] = {
        "lower": np.array(data['lower'], dtype=np.uint8),
        "upper": np.array(data['upper'], dtype=np.uint8),
        "draw_color": tuple(data['draw_color']),
        "led_color": tuple(data['led_color'])
    }

# ================= VARIABLES LEDS =================
NUM_LEDS_TOTAL = cfg_led['leds']['led_count']
LED_BRIGHTNESS = cfg_led['leds'].get('global_brightness', 31)

# Buscar el segmento para feedback (buscamos 'seg1', si no existe, fallback a 0-5)
FEEDBACK_SEGMENT = {'start': 0, 'end': 5} # Default

segments = cfg_led['leds'].get('segments', [])
target_seg = next((s for s in segments if s['name'] == 'seg1'), None)

if target_seg:
    # Ajuste: el YAML dice 'to: 16', asumimos inclusivo o exclusivo según preferencia.
    # Generalmente 'range(start, end)' en python es exclusivo al final.
    # Si tu YAML 'to' es inclusivo (led 16 se prende), sumamos 1.
    FEEDBACK_SEGMENT['start'] = target_seg['from']
    FEEDBACK_SEGMENT['end'] = target_seg['to'] + 1 
    print(f"[CONF] Usando segmento LED '{target_seg['name']}' (Indices {target_seg['from']}-{target_seg['to']})")
else:
    print("[CONF] No se encontró segmento 'seg1', usando primeros 5 LEDs.")


os.makedirs(SAVE_DIR, exist_ok=True)
logging.disable(logging.CRITICAL)

# ================= FUNCIONES AUXILIARES =================

def create_circular_mask(h, w, center, radius, invert=False):
    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y - center[1])**2)
    mask = dist_from_center <= radius
    if invert:
        mask = ~mask
    return (mask.astype(np.uint8) * 255)

def create_between_lines_mask(H, W, p1, p2, p3, p4):
    """
    Crea una máscara entre dos líneas definidas por
    (p1,p2) y (p3,p4)
    """
    mask = np.zeros((H, W), dtype=np.uint8)

    poly = np.array([
        p1,
        p2,
        p4,
        p3
    ], dtype=np.int32)

    cv2.fillPoly(mask, [poly], 255)

    return mask

def bbox_from_mask(mask_u8):
    ys, xs = np.nonzero(mask_u8)
    if len(xs) == 0:
        return None
    x0, x1 = xs.min(), xs.max() + 1
    y0, y1 = ys.min(), ys.max() + 1
    return x0, y0, x1, y1

def set_segment_color(leds, rgb):
    """Enciende solo el segmento configurado con el color dado, apaga el resto."""
    r, g, b = rgb
    start = FEEDBACK_SEGMENT['start']
    end = FEEDBACK_SEGMENT['end']
    
    for i in range(NUM_LEDS_TOTAL):
        if start <= i < end:
            leds.set_pixel(i, r, g, b, LED_BRIGHTNESS) # Pasamos brillo si la lib lo soporta
        else:
            leds.set_pixel(i, 0, 0, 0, 0)
    leds.show()

def find_largest_contour_by_color(hsv_image, lower, upper, min_area):
    mask = cv2.inRange(hsv_image, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0, mask
    best_cnt = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(best_cnt))
    if area < min_area:
        return None, 0, mask
    return best_cnt, area, mask

def detect_best_target(frame_bgr, min_area):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    best_target = None
    max_area_found = 0
    for color_name, props in COLORS_CONFIG.items():
        cnt, area, mask = find_largest_contour_by_color(hsv, props["lower"], props["upper"], min_area)
        if cnt is not None and area > max_area_found:
            max_area_found = area
            best_target = {
                "color_name": color_name,
                "contour": cnt,
                "area": area,
                "mask": mask,
                "draw_color": props["draw_color"],
                "led_color": props["led_color"]
            }
    return best_target

def contour_bbox_centroid(contour):
    x, y, w, h = cv2.boundingRect(contour)
    M = cv2.moments(contour)
    if M["m00"] > 1e-9:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx = x + w // 2
        cy = y + h // 2
    return x, y, w, h, cx, cy

def orientation_error_dist(cx, cy, W, H, ref="y-", offset=(0.0, 0.0)):
    x_center_img = (W - 1) / 2.0
    y_center_img = (H - 1) / 2.0
    dx, dy = offset
    x0 = x_center_img + dx
    y0 = y_center_img + dy
    vx = float(cx - x0)
    vy = float(cy - y0)
    if ref == "y-":
        theta = math.atan2(vx, -vy)
    elif ref == "y+":
        theta = math.atan2(-vx, vy)
    else:
        theta = 0.0
    dist_px = math.hypot(vx, vy)
    return (x0, y0), (vx, vy), theta, dist_px

def draw_vector_cv2(img, origin, vec, color=(255,255,0), thickness=2, scale=1.0):
    ox, oy = origin
    vx, vy = vec
    ex = int(ox + vx * scale)
    ey = int(oy + vy * scale)
    cv2.arrowedLine(img, (int(ox), int(oy)), (ex, ey), color, thickness, tipLength=0.2)

def draw_overlay_target(img_bgr, roi_top_left, target_data, W_full, H_full):
    x0, y0 = roi_top_left
    (origin_global, _, _, _) = orientation_error_dist(0, 0, W_full, H_full, offset=OFF_INNER)
    ox_g, oy_g = origin_global
    ox_local = ox_g - x0
    oy_local = oy_g - y0
    
    if 0 <= ox_local < img_bgr.shape[1] and 0 <= oy_local < img_bgr.shape[0]:
        cv2.drawMarker(img_bgr, (int(ox_local), int(oy_local)), (255, 255, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        cv2.putText(img_bgr, "BOT", (int(ox_local)+5, int(oy_local)+20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

    if target_data is not None:
        cnt = target_data["contour"]
        draw_col = target_data["draw_color"]
        color_name = target_data["color_name"]
        bx, by, bw, bh, cx, cy = contour_bbox_centroid(cnt)
        cv2.rectangle(img_bgr, (bx, by), (bx + bw, by + bh), draw_col, 2)
        
        cx_global = x0 + cx
        cy_global = y0 + cy
        _, (vx, vy), theta_rad, dist_px = orientation_error_dist(cx_global, cy_global, W_full, H_full, ref="y-", offset=OFF_INNER)
        theta_deg = math.degrees(theta_rad)

        if 0 <= ox_local < img_bgr.shape[1] and 0 <= oy_local < img_bgr.shape[0]:
            draw_vector_cv2(img_bgr, (ox_local, oy_local), (vx, vy), color=(0, 255, 255))
            
        info_text = f"{color_name} | D:{dist_px:.0f}px | E:{theta_deg:.1f}dg"
        text_size, _ = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        text_w, text_h = text_size
        cv2.rectangle(img_bgr, (bx, by - 25), (bx + text_w, by), (0,0,0), -1)
        cv2.putText(img_bgr, info_text, (bx, by - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

def main():
    # Inicialización LEDs usando el conteo total del YAML de LEDs
    leds = apa102.APA102(num_led=NUM_LEDS_TOTAL)
    set_segment_color(leds, (0,0,0)) # Apagar todo al inicio

    picam2 = Picamera2()
    
    config = picam2.create_video_configuration(
        main={"size": IMAGE_SIZE, "format": "RGB888"},
        controls={"AwbEnable": True, "AeEnable": True},
        transform=Transform(hflip=0, vflip=1)
    )
    picam2.configure(config)
    picam2.start()

    time.sleep(WARMUP_S)

    if LOCK_AWB_AE:
        try:
            picam2.set_controls({"AwbEnable": False, "AeEnable": False})
            print("[CAM] AE/AWB bloqueados.")
        except Exception as e:
            print(f"[CAM] No pude bloquear AE/AWB: {e}")

    # ================= GENERACIÓN DE MÁSCARA =================
    W, H = IMAGE_SIZE
    cx_base, cy_base = W / 2.0, H / 2.0

    center_outer = (cx_base + OFF_OUTER[0], cy_base + OFF_OUTER[1])
    ext = create_circular_mask(H, W, center=center_outer, radius=R_OUTER, invert=False)
    
    center_inner = (cx_base + OFF_INNER[0], cy_base + OFF_INNER[1])
    inte = create_circular_mask(H, W, center=center_inner, radius=R_INNER, invert=True)
    
    ring_mask = cv2.bitwise_and(ext, inte)
    
    # ================= MÁSCARAS ENTRE LINEAS =================

    if POLE_LINES is not None:

        # ---- PRIMER PAR DE LINEAS ----
        l1_p1 = tuple(POLE_LINES['line1']['p1'])
        l1_p2 = tuple(POLE_LINES['line1']['p2'])

        l2_p1 = tuple(POLE_LINES['line2']['p1'])
        l2_p2 = tuple(POLE_LINES['line2']['p2'])

        mask1 = create_between_lines_mask(
            H, W,
            l1_p1, l1_p2,
            l2_p1, l2_p2
        )

        ring_mask = cv2.bitwise_and(ring_mask, cv2.bitwise_not(mask1))


        # ---- SEGUNDO PAR DE LINEAS ----
        l3_p1 = tuple(POLE_LINES['line3']['p1'])
        l3_p2 = tuple(POLE_LINES['line3']['p2'])

        l4_p1 = tuple(POLE_LINES['line4']['p1'])
        l4_p2 = tuple(POLE_LINES['line4']['p2'])

        mask2 = create_between_lines_mask(
            H, W,
            l3_p1, l3_p2,
            l4_p1, l4_p2
        )

        ring_mask = cv2.bitwise_and(ring_mask, cv2.bitwise_not(mask2))
    
    bb = bbox_from_mask(ring_mask)
    if bb is None:
        raise RuntimeError("La máscara anillo salió vacía.")
    x0, y0, x1, y1 = bb
    ring_mask_roi = ring_mask[y0:y1, x0:x1]

    print(f"[TEST] Configs: {CAM_CONFIG_PATH} & {LED_CONFIG_PATH}")
    print(f"[TEST] ROI: {x1-x0}x{y1-y0} | Guardando en: '{SAVE_DIR}'")

    led_state = False
    current_led_color = None
    t0 = time.time()
    frames = 0
    last_save_t = time.time()

    try:
        while True:
            frame_bgr = picam2.capture_array("main")
            roi_raw_bgr = frame_bgr[y0:y1, x0:x1]
            roi_masked_bgr = cv2.bitwise_and(roi_raw_bgr, roi_raw_bgr, mask=ring_mask_roi)

            target = detect_best_target(roi_masked_bgr, MIN_AREA)
            has_target = (target is not None)

            dist_px = 0.0
            theta_deg = 0.0
            tgt_name = "NONE"
            
            if has_target:
                bx, by, bw, bh, cx, cy = contour_bbox_centroid(target["contour"])
                cx_global = x0 + cx
                cy_global = y0 + cy
                _, (vx, vy), theta_rad, dist_px = orientation_error_dist(cx_global, cy_global, W, H, ref="y-", offset=OFF_INNER)
                theta_deg = math.degrees(theta_rad)
                tgt_name = target["color_name"]

            # --- GUARDADO ---
            now = time.time()
            if (now - last_save_t) >= SAVE_EVERY_S:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                raw_path = os.path.join(SAVE_DIR, f"{ts}_raw.jpg")
                masked_path = os.path.join(SAVE_DIR, f"{ts}_masked.jpg")
                
                annotated_raw = frame_bgr.copy()
                masked_full = cv2.bitwise_and(frame_bgr, frame_bgr, mask=ring_mask)
                
                target_global = None
                if has_target:
                    target_global = target.copy()
                    cnt_gl = target["contour"].copy()
                    cnt_gl[:, :, 0] += x0
                    cnt_gl[:, :, 1] += y0
                    target_global["contour"] = cnt_gl
                
                draw_overlay_target(annotated_raw, (0, 0), target_global, W, H)
                cv2.circle(annotated_raw, (int(center_outer[0]), int(center_outer[1])), int(R_OUTER), (255, 255, 0), 1)
                cv2.circle(annotated_raw, (int(center_inner[0]), int(center_inner[1])), int(R_INNER), (255, 0, 255), 1)
                
                if POLE_LINES is not None:
                    cv2.line(annotated_raw, l1_p1, l1_p2, (0,255,255), 2)
                    cv2.line(annotated_raw, l2_p1, l2_p2, (0,255,255), 2)
                    cv2.line(annotated_raw, l3_p1, l3_p2, (0,255,255), 2)
                    cv2.line(annotated_raw, l4_p1, l4_p2, (0,255,255), 2)
                
                draw_overlay_target(masked_full, (0, 0), target_global, W, H)
                cv2.circle(masked_full, (int(center_outer[0]), int(center_outer[1])), int(R_OUTER), (255, 255, 0), 1)
                cv2.circle(masked_full, (int(center_inner[0]), int(center_inner[1])), int(R_INNER), (255, 0, 255), 1)

                cv2.imwrite(raw_path, annotated_raw)
                cv2.imwrite(masked_path, masked_full)
                last_save_t = now

            # --- CONTROL LED (Usando set_segment_color) ---
            if has_target:
                tgt_color = target["led_color"]
                if (not led_state) or (current_led_color != tgt_color):
                    set_segment_color(leds, tgt_color)
                    led_state = True
                    current_led_color = tgt_color
            elif (not has_target) and led_state:
                set_segment_color(leds, (0,0,0))
                led_state = False
                current_led_color = None

            frames += 1
            if frames % 30 == 0:
                fps = frames / (time.time() - t0)
                status = f"TGT={tgt_name} | Dist={dist_px:.1f}px" if has_target else "SEARCHING..."
                print(f"[STATS] FPS={fps:.1f} | {status}")

    except KeyboardInterrupt:
        print("\n[STOP] Usuario detuvo el script.")
    finally:
        set_segment_color(leds, (0,0,0))
        leds.cleanup()
        picam2.stop()

if __name__ == "__main__":
    main()