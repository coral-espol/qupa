#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gpio_led_node — ROS2 Jazzy node for the UV LED bank (GPIO pin 26).

Drives a transistor on a single GPIO pin to switch the UV LEDs on or off.
A service is used instead of a topic because the command is discrete and
callers benefit from knowing it was applied.

Services:
  ~/set   std_srvs/SetBool   True = on, False = off
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

try:
    from gpiozero import LED as GPIOLED
    _HW = True
except ImportError:
    _HW = False


class GpioLedNode(Node):

    def __init__(self):
        super().__init__('uv_led')

        self.declare_parameter('pin', 26)
        pin = self.get_parameter('pin').value

        self._led = None
        if not _HW:
            self.get_logger().error(
                'gpiozero not found — install with: pip install gpiozero'
            )
        else:
            try:
                self._led = GPIOLED(pin)
                self._led.on()   # active-low: pin high = LEDs off
                self.get_logger().info(f'UV LED ready on GPIO pin {pin}.')
            except Exception as e:
                self.get_logger().error(f'UV LED init failed on pin {pin}: {e}')

        self.create_service(SetBool, '~/set', self._set_cb)
        self.get_logger().info(f'UV LED service ready on {self.get_namespace()}/uv_led/set')

    def _set_cb(self, request: SetBool.Request,
                response: SetBool.Response):
        if self._led is not None:
            # active-low: request True (on) → pin low; False (off) → pin high
            self._led.off() if request.data else self._led.on()

        response.success = True
        response.message = 'on' if request.data else 'off'
        return response

    def destroy_node(self):
        if self._led is not None:
            self._led.on()   # active-low: pin high = LEDs off on shutdown
            self._led.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
