#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ir_scanner_node — ROS2 Jazzy node for GP2Y0E03 IR proximity sensors.

Publishes to /scan as std_msgs/Float32MultiArray with 6 distance values (cm).
Special values:
  -1.0  → sensor saturated (object too close / out of range)
  -2.0  → I2C read error / channel not responding
"""

import time
import statistics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


# ── Sensor registers (GP2Y0E03) ──────────────────────────────────────────────
REG_SHIFT  = 0x35
REG_DIST_H = 0x5E
REG_DIST_L = 0x5F

# Channels kept after filtering (indices into active_channels list)
# active_channels = [0,1,2,3,4,5,6,7] → we keep channels 1,2,3,5,6,7
KEEP_CHANNELS = [1, 2, 3, 5, 6, 7]


# ── Calibration helper ───────────────────────────────────────────────────────

def apply_calibration(channel: int, raw_cm: float, cal_channels: dict,
                       cal_range: list) -> float:
    """Apply per-channel quadratic calibration: dist = a*x² + b*x + c."""
    if channel not in cal_channels:
        return raw_cm

    coeffs = cal_channels[channel]
    a, b, c = coeffs['a'], coeffs['b'], coeffs['c']
    dist = a * raw_cm ** 2 + b * raw_cm + c

    min_d, max_d = cal_range
    return max(min_d, min(max_d, dist))


# ── Low-level sensor driver ───────────────────────────────────────────────────

class IRScanner:
    """Drives up to 8 GP2Y0E03 sensors behind a TCA9548A I2C multiplexer."""

    def __init__(self, mux_addr: int, sensor_addr: int,
                 settle_s: float, samples: int, sample_delay_s: float,
                 cal_channels: dict, cal_range: list):
        self.bus         = smbus2.SMBus(1)
        self.mux_addr    = mux_addr
        self.sensor_addr = sensor_addr
        self.settle_s    = settle_s
        self.samples     = samples
        self.sample_delay_s = sample_delay_s
        self.cal_channels   = cal_channels
        self.cal_range      = cal_range
        self._shift_cache   = {}

    def deselect_all(self):
        try:
            self.bus.write_byte(self.mux_addr, 0x00)
        except OSError:
            pass

    def _select_channel(self, channel: int) -> bool:
        try:
            self.bus.write_byte(self.mux_addr, 1 << channel)
            time.sleep(self.settle_s)
        except OSError:
            return False

        if channel not in self._shift_cache:
            try:
                shift = self.bus.read_byte_data(self.sensor_addr, REG_SHIFT)
                self._shift_cache[channel] = shift
            except OSError:
                return False

        return True

    def _read_raw_distance(self, channel: int):
        """Return distance in cm, -1.0 on saturation, None on I2C error."""
        try:
            shift = self._shift_cache[channel]
            data  = self.bus.read_i2c_block_data(self.sensor_addr, REG_DIST_H, 2)
            hi, lo = data[0], data[1]
            raw12  = ((hi << 8) | lo) >> 4

            if hi == 0xFF and (lo & 0x0F) == 0x0F:
                return -1.0  # saturated

            dist_mm = raw12 / (2 ** shift)
            return round(dist_mm / 10.0, 2)
        except OSError:
            return None

    def get_distance(self, channel: int) -> float:
        """
        Return calibrated distance in cm for *channel*.
        Returns -1.0 (saturated) or -2.0 (I2C error) on failure.
        """
        if not _select_channel_safe(self, channel):
            return -2.0

        # Discard first reading (settling artefact)
        self._read_raw_distance(channel)

        valid     = []
        sat_count = 0

        for _ in range(self.samples):
            d = self._read_raw_distance(channel)
            if d is None:
                pass
            elif d < 0:
                sat_count += 1
            else:
                valid.append(d)
            time.sleep(self.sample_delay_s)

        if valid:
            raw = statistics.median(valid)
            return apply_calibration(channel, raw, self.cal_channels,
                                     self.cal_range)
        if sat_count > 0:
            return -1.0

        return -2.0


def _select_channel_safe(scanner: IRScanner, channel: int) -> bool:
    return scanner._select_channel(channel)


# ── ROS2 Node ────────────────────────────────────────────────────────────────

class IRScannerNode(Node):

    def __init__(self):
        super().__init__('ir_scanner')

        # ── Parameters (can be overridden via launch / param file) ──
        self.declare_parameter('mux_address',    0x70)
        self.declare_parameter('sensor_address', 0x40)
        self.declare_parameter('samples',        3)
        self.declare_parameter('settle_s',       0.001)
        self.declare_parameter('sample_delay_s', 0.001)
        self.declare_parameter('loop_dt',        0.01)
        self.declare_parameter('cal_range',      [4.0, 50.0])

        # Calibration coefficients — flat list: [ch, a, b, c, ch, a, b, c, ...]
        # Default matches sensor.yaml channels 1,2,3,5,6,7
        default_coeffs = [
            1, -0.0006299,  0.5671630,  0.6585924,
            2, -0.0009288,  0.6876107, -1.3613167,
            3, -0.0016829,  0.7494877, -1.8706732,
            5,  0.0002581,  0.5470853,  1.0180294,
            6, -0.0019746,  0.7568900, -1.4506072,
            7, -0.0009400,  0.6889223, -0.7839605,
        ]
        self.declare_parameter('calibration_coeffs', default_coeffs)

        mux_addr    = self.get_parameter('mux_address').value
        sensor_addr = self.get_parameter('sensor_address').value
        samples     = self.get_parameter('samples').value
        settle_s    = self.get_parameter('settle_s').value
        sample_delay_s = self.get_parameter('sample_delay_s').value
        self._loop_dt  = self.get_parameter('loop_dt').value
        cal_range      = list(self.get_parameter('cal_range').value)

        raw_coeffs  = list(self.get_parameter('calibration_coeffs').value)
        cal_channels = self._parse_coeffs(raw_coeffs)

        # ── Publisher ──
        self._pub = self.create_publisher(Float32MultiArray, 'scan', 10)

        # ── Scanner ──
        if not SMBUS_AVAILABLE:
            self.get_logger().error('smbus2 not found — cannot open I2C bus.')
            return

        try:
            self._scanner = IRScanner(
                mux_addr=mux_addr,
                sensor_addr=sensor_addr,
                settle_s=settle_s,
                samples=samples,
                sample_delay_s=sample_delay_s,
                cal_channels=cal_channels,
                cal_range=cal_range,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open I2C bus: {e}')
            return

        self._timer = self.create_timer(self._loop_dt, self._scan_callback)
        self.get_logger().info(
            f'IR Scanner ready — publishing {len(KEEP_CHANNELS)} sensors '
            f'on /scan at {1.0/self._loop_dt:.1f} Hz'
        )

    # ── Helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_coeffs(flat: list) -> dict:
        """Convert [ch, a, b, c, ...] flat list to {ch: {a,b,c}} dict."""
        result = {}
        it = iter(flat)
        for ch, a, b, c in zip(it, it, it, it):
            result[int(ch)] = {'a': float(a), 'b': float(b), 'c': float(c)}
        return result

    # ── Timer callback ────────────────────────────────────────────────────────

    def _scan_callback(self):
        distances = []
        for ch in KEEP_CHANNELS:
            dist = self._scanner.get_distance(ch)
            distances.append(float(dist))

        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='sensors', size=len(distances), stride=len(distances))
        ]
        msg.data = distances

        self._pub.publish(msg)
        self.get_logger().debug(str(distances))

    def destroy_node(self):
        if hasattr(self, '_scanner'):
            self._scanner.deselect_all()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = IRScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
