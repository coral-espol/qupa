# Calibración de distancia y ángulo (cámara espejo)

Convierte las detecciones en píxeles del robot (`camera/detections`) en distancia (m) y ángulo (rad) respecto al centro del robot (`base_link`).

- El **robot** solo publica el centroide del blob (`cx`, `cy`). No hay que cambiar nada en él al recalibrar.
- La **PC** ajusta y aplica el modelo: centro óptico, offset y sentido del ángulo, y la curva `r_px → d_m`.

Requisito previo: la máscara ya está calibrada ([calibrate_mask.md](calibrate_mask.md)) y el objetivo se detecta de forma estable.

---

## Convenciones

| Magnitud | Referencia |
|---|---|
| Distancia | Del **centro del robot observador** (debajo del espejo) al **centro del objetivo** |
| Ángulo | 0° = frente del robot, **positivo hacia la izquierda** (antihorario, REP-103) |

El objetivo debe ser **el mismo objeto que se usará en los experimentos**, por ejemplo el robot líder con su marca de color. La curva depende de la altura de la marca, así que si esa altura cambia hay que recalibrar.

---

## Preparación en el laboratorio (Byron)

1. Pegar en el piso una plantilla polar centrada en el robot: líneas cada 45° y marcas a 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.80 y 1.00 m.
2. Poner el robot en el centro, **alineado con 0°**. No se mueve durante toda la sesión.
3. Usar la misma iluminación que tendrán los experimentos.
4. En el robot:
   ```bash
   ros2 launch qupa_hardware camera.launch.py namespace:=qupa_3A
   ```

---

## Grabación (desde la PC, en remoto)

```bash
ros2 run qupa_desktop calibrate_range record --ns qupa_3A --csv calib_3A.csv --color BLUE
```

Byron coloca el objetivo y avisa. Tú escribes `p <distancia_m> <ángulo_deg>`. La herramienta promedia 9 frames (unos 3 s) y guarda el punto en el CSV. Si el programa se cierra, no se pierde nada.

Comando `s`: muestra qué se está viendo. Comando `u`: deshace el último punto.

**Orden recomendado (unos 25 puntos, 20–30 min):**

| Bloque | Puntos | Para qué sirve |
|---|---|---|
| A. Círculo | 0.30 m en 0, 45, 90, …, 315° (8 puntos) | Centro óptico, offset y sentido del ángulo |
| B. Radial | 0° en todas las distancias de la plantilla | Curva `r_px → d_m` |
| C. Radial extra | 90° y 180° en 0.20, 0.50 y 0.80 m | Comprobar que la curva no depende del ángulo |

Nota: a 180° el objetivo puede quedar tapado por un poste. Si no se detecta, sáltalo.

Después, en un **CSV aparte** (`valid_3A.csv`), graba unos 10 puntos en posiciones **distintas** a las anteriores, por ejemplo 0.25 m a 30° o 0.70 m a −120°. Son los datos de validación para el paper.

---

## Ajuste y validación

```bash
ros2 run qupa_desktop calibrate_range fit calib_3A.csv \
    --out ~/qupa_ros/src/qupa/qupa_desktop/config/vision_qupa_3A.yaml
ros2 run qupa_desktop calibrate_range eval \
    ~/qupa_ros/src/qupa/qupa_desktop/config/vision_qupa_3A.yaml valid_3A.csv
```

`fit` hace tres cosas:
- Estima el centro con un ajuste de círculo (bloque A).
- Calcula el offset y el sentido del ángulo.
- Elige la curva de distancia (polinomio o log-polinomio, grado 1 a 3) según el error *leave-one-out*.

Imprime el RMSE y guarda una gráfica PNG. `eval` calcula el RMSE sobre los puntos de validación: **ese es el número que va al paper**.

---

## Uso

```bash
cd ~/qupa_ros && colcon build --packages-select qupa_desktop && source install/setup.bash
ros2 launch qupa_desktop vision.launch.py namespace:=qupa_3A
ros2 topic echo /qupa_3A/camera/targets
```

En RViz, agrega un display `MarkerArray` en `/qupa_3A/camera/targets_viz` para ver los objetivos como cilindros. Los que están fuera del rango calibrado aparecen semitransparentes.

---

## Qué esperar

El espejo comprime las distancias lejanas: a partir de ~0.6 m, 1 px de ruido puede equivaler a varios cm. Al grabar, revisa el `±` que muestra cada punto. Si pasa de 3 px (la herramienta avisa con `¡ruidoso!`), la detección es inestable: revisa la iluminación o el HSV antes de culpar al modelo.
