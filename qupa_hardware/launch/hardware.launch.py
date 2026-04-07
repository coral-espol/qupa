from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('qupa_hardware')
    ir_cfg    = os.path.join(pkg, 'config', 'ir_scanner.yaml')
    motor_cfg = os.path.join(pkg, 'config', 'motor.yaml')
    floor_cfg = os.path.join(pkg, 'config', 'floor_sensor.yaml')
    leds_cfg  = os.path.join(pkg, 'config', 'leds.yaml')

    return LaunchDescription([

        # IR scanner — 10 Hz, publishes /qupa_3A/scan in base_link frame
        Node(
            package='qupa_hardware',
            executable='ir_scanner',
            name='ir_scanner',
            namespace='qupa_3A',
            output='screen',
            parameters=[ir_cfg],
        ),

        # Motor driver — subscriber-driven on /qupa_3A/cmd_vel, watchdog at 20 Hz
        Node(
            package='qupa_hardware',
            executable='motor_driver',
            name='motor_node',
            namespace='qupa_3A',
            output='screen',
            parameters=[motor_cfg],
        ),

        # Floor colour sensor — TCS34725, publishes /qupa_3A/floor/color
        Node(
            package='qupa_hardware',
            executable='floor_sensor',
            name='floor_sensor_node',
            namespace='qupa_3A',
            output='screen',
            parameters=[floor_cfg],
        ),

        # LED strip — APA102, subscribes to /qupa_3A/leds/command
        Node(
            package='qupa_hardware',
            executable='led',
            name='led_node',
            namespace='qupa_3A',
            output='screen',
            parameters=[leds_cfg],
        ),

    ])
