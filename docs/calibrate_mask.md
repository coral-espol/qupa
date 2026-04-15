# Calibración visual de máscara de cámara

Herramienta gráfica para definir los parámetros de exclusión de pilares y el offset del centro del espejo sin tener que editar coordenadas a mano.

Corre en la **PC** a partir de una foto tomada con la cámara del robot.

---

## Requisitos

```bash
pip install opencv-python
```

---

## Paso 1 — Tomar la foto en el robot

La foto debe tener el mismo tamaño que usa el nodo de cámara (640×480):

```bash
rpicam-still -o foto.jpg --nopreview --width 640 --height 480
```

Copiar a la PC:

```bash
scp qupa@192.168.0.120:~/foto.jpg .
```

---

## Paso 2 — Correr la herramienta

```bash
ros2 run qupa_desktop calibrate_mask foto.jpg
```

---

## Paso 3 — Marcar los puntos

La herramienta guía el proceso en 3 pasos:

### 1. Centro del espejo
Click en el centro del círculo del espejo.
Genera `inner_offset_x`, `inner_offset_y`, `outer_offset_x`, `outer_offset_y`.

### 2. Circulo interno
Mueva el mouse para generar una circunferencia interna que se adapte a la geometria deseada

### 3. Circulo externo
Mueva el mouse para generar una circunferencia externa que se adapte a la geometria deseada

### 2. Pilar 1
Click en las 4 esquinas del área a excluir (el pilar o soporte izquierdo).
Primero seleccione el punto de la circunferencia interna y luego la externa de la parte superior. Repita el mismo orden para la inferior.


### 3. Pilar 2
Click en las 4 esquinas del área a excluir (el pilar o soporte derecho).
Primero seleccione el punto de la circunferencia interna y luego la externa de la parte superior. Repita el mismo orden para la inferior.


### Resultado
Presiona cualquier tecla al terminar. El YAML se imprime en la terminal:

```
──────────────────────────────────────────────────
# Pega estos valores en camera.yaml (sección camera_node)
──────────────────────────────────────────────────

    inner_offset_x:  -18
    inner_offset_y:  14
    outer_offset_x:  -18
    outer_offset_y:  14

    pole_line1_p1: [0, 185]
    pole_line1_p2: [220, 215]
    pole_line2_p1: [0, 255]
    pole_line2_p2: [220, 245]

    pole_line3_p1: [420, 215]
    pole_line3_p2: [640, 185]
    pole_line4_p1: [420, 245]
    pole_line4_p2: [640, 255]
```

---

## Controles

| Tecla | Acción |
|---|---|
| Click izquierdo | Agregar punto |
| `Z` | Deshacer último punto |
| `R` | Reiniciar todo |
| `Q` | Salir sin guardar |

---

## Paso 4 — Aplicar los valores

Copia los valores impresos en `qupa_hardware/config/camera.yaml` bajo la sección `camera_node:`:

```yaml
camera_node:
  ros__parameters:
    inner_offset_x:  -18
    inner_offset_y:  14
    outer_offset_x:  -18
    outer_offset_y:  14

    pole_line1_p1: [0,   185]
    pole_line1_p2: [220, 215]
    pole_line2_p1: [0,   255]
    pole_line2_p2: [220, 245]
    pole_line3_p1: [640, 185]
    pole_line3_p2: [420, 215]
    pole_line4_p1: [640, 255]
    pole_line4_p2: [420, 245]
```

Luego reconstruir en el robot:

```bash
colcon build --packages-select qupa_hardware --executor sequential --parallel-workers 1
```

---

## Notas

- Los parámetros de `camera_node` y `camera_calibration_node` son compartidos — editar el YAML una vez afecta a ambos nodos.
- Para verificar visualmente el resultado, lanzar el nodo de calibración y ver la imagen en RViz: `ros2 launch qupa_hardware camera_calibration.launch.py`
