#!/usr/bin/env python3
import time
import statistics
import smbus2
import numpy as np

# ── CONFIG ─────────────────────────────────────────────
CHANNELS = [1, 2, 3, 5, 6, 7]

CHANNEL_LABEL = {
    1: "S",
    2: "W",
    3: "NW",
    5: "N",
    6: "NE",
    7: "E",
}

REG_SHIFT  = 0x35
REG_DIST_H = 0x5E

# ── DRIVER (RAW) ───────────────────────────────────────
class IRScanner:
    def __init__(self, mux_addr=0x70, sensor_addr=0x40):
        self.bus = smbus2.SMBus(1)
        self.mux_addr = mux_addr
        self.sensor_addr = sensor_addr
        self._shift_cache = {}

    def select_channel(self, ch):
        self.bus.write_byte(self.mux_addr, 1 << ch)
        time.sleep(0.002)

        if ch not in self._shift_cache:
            shift = self.bus.read_byte_data(self.sensor_addr, REG_SHIFT)
            self._shift_cache[ch] = shift

    def read_raw_cm(self, ch):
        try:
            self.select_channel(ch)

            shift = self._shift_cache[ch]
            data = self.bus.read_i2c_block_data(self.sensor_addr, REG_DIST_H, 2)

            hi, lo = data
            raw12 = ((hi << 8) | lo) >> 4

            if hi == 0xFF and (lo & 0x0F) == 0x0F:
                return None  # saturado

            dist_mm = raw12 / (2 ** shift)
            return dist_mm / 10.0  # cm

        except:
            return None

    def get_stable_raw(self, ch, samples=5):
        vals = []
        for _ in range(samples):
            d = self.read_raw_cm(ch)
            if d is not None:
                vals.append(d)
            time.sleep(0.01)

        if not vals:
            return None

        return statistics.median(vals)

# ── CALIBRACIÓN ───────────────────────────────────────
def calibrate_channel(scanner, ch):
    print(f"\n==============================")
    print(f"Calibrando sensor ch{ch} ({CHANNEL_LABEL[ch]})")
    print(f"==============================")

    raw_list = []
    ref_list = []

    while True:
        cmd = input("\nIngresa distancia de referencia en cm (o 'q' para terminar): ")

        if cmd.lower() == 'q':
            break

        try:
            ref = float(cmd)
        except:
            print("Valor inválido")
            continue

        input("Coloca el robot y presiona ENTER para capturar...")

        raw = scanner.get_stable_raw(ch)

        if raw is None:
            print("Error leyendo sensor")
            continue

        print(f"✔ RAW: {raw:.3f} cm | REF: {ref:.3f} cm")

        raw_list.append(raw)
        ref_list.append(ref)

    if len(raw_list) < 3:
        print("No hay suficientes puntos (mínimo 3)")
        return None

    # ajuste cuadrático
    coeffs = np.polyfit(raw_list, ref_list, 2)

    return coeffs  # [a, b, c]

# ── MAIN ──────────────────────────────────────────────
def main():
    scanner = IRScanner()
    results = {}

    for ch in CHANNELS:
        coeffs = calibrate_channel(scanner, ch)

        if coeffs is not None:
            results[ch] = coeffs

    print("\n\n======= RESULTADOS =======\n")

    for ch in CHANNELS:
        if ch in results:
            a, b, c = results[ch]
            print(
                f"cal_ch{ch}: [{a: .7f}, {b: .7f}, {c: .7f}]  # {CHANNEL_LABEL[ch]}"
            )

if __name__ == "__main__":
    main()