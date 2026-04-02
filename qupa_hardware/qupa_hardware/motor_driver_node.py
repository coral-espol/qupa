#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_driver_node — ROS2 Jazzy node for differential-drive motor control.

Subscribes to geometry_msgs/Twist on 'cmd_vel', converts to per-wheel PWM
via differential kinematics, and drives two motors through RPi.GPIO.
A watchdog timer stops the motors if no command arrives within cmd_timeout_s.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


# ── Motor hardware driver ─────────────────────────────────────────────────────

class MotorHardware:
    """Low-level PWM driver for two DC motors via RPi.GPIO."""

    def __init__(self, pin_in1_l: int, pin_in2_l: int,
                 pin_in1_r: int, pin_in2_r: int, pwm_freq: int):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self._left_fwd  = pin_in1_l
        self._left_rev  = pin_in2_l
        self._right_fwd = pin_in1_r
        self._right_rev = pin_in2_r

        GPIO.setup(
            [self._left_fwd, self._left_rev, self._right_fwd, self._right_rev],
            GPIO.OUT,
        )

        self._pwm_lf = GPIO.PWM(self._left_fwd,  pwm_freq)
        self._pwm_lr = GPIO.PWM(self._left_rev,  pwm_freq)
        self._pwm_rf = GPIO.PWM(self._right_fwd, pwm_freq)
        self._pwm_rr = GPIO.PWM(self._right_rev, pwm_freq)

        for p in (self._pwm_lf, self._pwm_lr, self._pwm_rf, self._pwm_rr):
            p.start(0)

    def apply_power(self, left_pwm: float, right_pwm: float):
        """Apply signed duty cycle [-100..100] to each motor."""
        left_pwm  = max(-100.0, min(100.0, float(left_pwm)))
        right_pwm = max(-100.0, min(100.0, float(right_pwm)))

        if left_pwm >= 0:
            self._pwm_lf.ChangeDutyCycle(left_pwm)
            self._pwm_lr.ChangeDutyCycle(0)
        else:
            self._pwm_lf.ChangeDutyCycle(0)
            self._pwm_lr.ChangeDutyCycle(-left_pwm)

        if right_pwm >= 0:
            self._pwm_rf.ChangeDutyCycle(right_pwm)
            self._pwm_rr.ChangeDutyCycle(0)
        else:
            self._pwm_rf.ChangeDutyCycle(0)
            self._pwm_rr.ChangeDutyCycle(-right_pwm)

    def stop(self):
        for p in (self._pwm_lf, self._pwm_lr, self._pwm_rf, self._pwm_rr):
            p.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        # ── Parameters ──
        # Hardware
        self.declare_parameter('pins.IN1_L',  6)
        self.declare_parameter('pins.IN2_L',  5)
        self.declare_parameter('pins.IN1_R',  13)
        self.declare_parameter('pins.IN2_R',  19)
        self.declare_parameter('pwm_freq',    1000)
        self.declare_parameter('duty_min',    0.0)
        self.declare_parameter('duty_max',    100.0)

        # Kinematics / control
        self.declare_parameter('wheel_base',    0.086)
        self.declare_parameter('v_max_mps',     0.08)
        self.declare_parameter('w_max_rps',     2.50)
        self.declare_parameter('pwm_min',       25.0)
        self.declare_parameter('pwm_max',       90.0)
        self.declare_parameter('deadband',      0.03)
        self.declare_parameter('cmd_timeout_s', 0.3)

        pin_in1_l = self.get_parameter('pins.IN1_L').value
        pin_in2_l = self.get_parameter('pins.IN2_L').value
        pin_in1_r = self.get_parameter('pins.IN1_R').value
        pin_in2_r = self.get_parameter('pins.IN2_R').value
        pwm_freq  = self.get_parameter('pwm_freq').value

        self._L        = self.get_parameter('wheel_base').value
        self._v_max    = self.get_parameter('v_max_mps').value
        self._w_max    = self.get_parameter('w_max_rps').value
        self._pwm_min  = float(self.get_parameter('pwm_min').value)
        self._pwm_max  = float(self.get_parameter('pwm_max').value)
        self._deadband = self.get_parameter('deadband').value
        self._duty_min = float(self.get_parameter('duty_min').value)
        self._duty_max = float(self.get_parameter('duty_max').value)
        self._timeout  = self.get_parameter('cmd_timeout_s').value

        self._last_cmd = None   # rclpy.time.Time | None

        # ── Hardware ──
        if not GPIO_AVAILABLE:
            self.get_logger().error('RPi.GPIO not found — cannot init motors.')
            return

        try:
            self._hw = MotorHardware(
                pin_in1_l, pin_in2_l, pin_in1_r, pin_in2_r, pwm_freq
            )
        except Exception as e:
            self.get_logger().error(f'Motor hardware init failed: {e}')
            return

        # ── Subscriber + watchdog ──
        self._sub      = self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self._watchdog = self.create_timer(0.05, self._watchdog_cb)

        self.get_logger().info(
            f'Motor driver ready — '
            f'v_max={self._v_max} m/s, w_max={self._w_max} rad/s, L={self._L} m | '
            f'PWM [{self._pwm_min}..{self._pwm_max}], deadband={self._deadband}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clamp(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(x)))

    def _vel_to_pwm(self, v_wheel_mps: float) -> float:
        """Map wheel linear velocity (m/s) → signed duty cycle [-100..100]."""
        v = float(v_wheel_mps)
        if abs(v) < self._deadband:
            return 0.0
        sign  = 1.0 if v >= 0 else -1.0
        v_abs = min(abs(v), self._v_max)
        pwm   = self._pwm_min + (v_abs / self._v_max) * (self._pwm_max - self._pwm_min)
        return sign * self._clamp(pwm, self._duty_min, self._duty_max)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        v = self._clamp(msg.linear.x,  -self._v_max, self._v_max)
        w = self._clamp(msg.angular.z, -self._w_max, self._w_max)

        v_l = v - w * self._L / 2.0
        v_r = v + w * self._L / 2.0

        self._hw.apply_power(self._vel_to_pwm(v_l), self._vel_to_pwm(v_r))
        self._last_cmd = self.get_clock().now()

    def _watchdog_cb(self):
        if self._last_cmd is None:
            return
        elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if elapsed > self._timeout:
            self._hw.stop()
            self._last_cmd = None

    def destroy_node(self):
        if hasattr(self, '_hw'):
            self._hw.cleanup()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
