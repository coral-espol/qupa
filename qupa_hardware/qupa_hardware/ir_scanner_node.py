#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ir_scanner_node — ROS2 Jazzy node for GP2Y0E03 IR proximity sensors.

Publishes one sensor_msgs/LaserScan per sensor on individual topics,
each stamped with the sensor's TF frame from qupa_description.

  Channel | Frame   | Direction
  --------+---------+----------
    ch1   | ir_270  |   S
    ch2   | ir_180  |   W
    ch3   | ir_135  |   NW
    ch4   | ir_90   |   N
    ch5   | ir_45   |   NE
    ch6   | ir_0    |   E
"""

import time
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


# ── Sensor registers (GP2Y0E03) ──────────────────────────────────────────────
REG_SHIFT  = 0x35
REG_DIST_H = 0x5E
REG_DIST_L = 0x5F

# Mapping: (channel, tf_frame, topic_suffix)
SENSOR_MAP = [
    (1, 'ir_270', 'ir_s'),
    (2, 'ir_180', 'ir_w'),
    (3, 'ir_135', 'ir_nw'),
    (4, 'ir_90',  'ir_n'),
    (5, 'ir_45',  'ir_ne'),
    (6, 'ir_0',   'ir_e'),
]

KEEP_CHANNELS = [ch for ch, _, _ in SENSOR_MAP]


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
        self.bus            = smbus2.SMBus(1)
        self.mux_addr       = mux_addr
        self.sensor_addr    = sensor_addr
        self.settle_s       = settle_s
        self.samples        = samples
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
        if not self._select_channel(channel):
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


# ── ROS2 Node ────────────────────────────────────────────────────────────────

class IRScannerNode(Node):

    def __init__(self):
        super().__init__('ir_scanner')

        # ── Parameters ──
        self.declare_parameter('mux_address',    0x70)
        self.declare_parameter('sensor_address', 0x40)
        self.declare_parameter('samples',        3)
        self.declare_parameter('settle_s',       0.001)
        self.declare_parameter('sample_delay_s', 0.001)
        self.declare_parameter('loop_dt',        0.1)
        self.declare_parameter('range_min_m',    0.04)   # 4 cm
        self.declare_parameter('range_max_m',    0.50)   # 50 cm
        self.declare_parameter('cal_range',      [4.0, 50.0])

        # Calibration coefficients — one [a, b, c] list per channel
        self.declare_parameter('cal_ch1', [-0.0006299,  0.5671630,  0.6585924])  # S
        self.declare_parameter('cal_ch2', [-0.0009288,  0.6876107, -1.3613167])  # W
        self.declare_parameter('cal_ch3', [-0.0016829,  0.7494877, -1.8706732])  # NW
        self.declare_parameter('cal_ch4', [ 0.0002581,  0.5470853,  1.0180294])  # N
        self.declare_parameter('cal_ch5', [-0.0019746,  0.7568900, -1.4506072])  # NE
        self.declare_parameter('cal_ch6', [-0.0009400,  0.6889223, -0.7839605])  # E

        mux_addr       = self.get_parameter('mux_address').value
        sensor_addr    = self.get_parameter('sensor_address').value
        samples        = self.get_parameter('samples').value
        settle_s       = self.get_parameter('settle_s').value
        sample_delay_s = self.get_parameter('sample_delay_s').value
        self._loop_dt  = self.get_parameter('loop_dt').value
        self._range_min = self.get_parameter('range_min_m').value
        self._range_max = self.get_parameter('range_max_m').value
        cal_range      = list(self.get_parameter('cal_range').value)

        cal_channels = {
            ch: dict(zip('abc', self.get_parameter(f'cal_ch{ch}').value))
            for ch in KEEP_CHANNELS
        }

        # ── Publishers — one per sensor ──
        self._publishers = {
            ch: self.create_publisher(LaserScan, topic, 10)
            for ch, _, topic in SENSOR_MAP
        }
        self._frames = {ch: frame for ch, frame, _ in SENSOR_MAP}

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
            f'IR Scanner ready — publishing 6 LaserScan topics '
            f'at {1.0/self._loop_dt:.1f} Hz'
        )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _scan_callback(self):
        now = self.get_clock().now().to_msg()

        for ch, pub in self._publishers.items():
            dist_cm = self._scanner.get_distance(ch)
            distance_m = dist_cm / 100.0 if dist_cm > 0 else self._range_max

            msg = LaserScan()
            msg.header.stamp    = now
            msg.header.frame_id = self._frames[ch]
            msg.angle_min       = 0.0
            msg.angle_max       = 0.0
            msg.angle_increment = 0.0
            msg.time_increment  = 0.0
            msg.scan_time       = self._loop_dt
            msg.range_min       = self._range_min
            msg.range_max       = self._range_max
            msg.ranges          = [distance_m]

            pub.publish(msg)

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
