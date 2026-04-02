from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('qupa_hardware')
    ir_cfg    = os.path.join(pkg, 'config', 'ir_scanner.yaml')
    motor_cfg = os.path.join(pkg, 'config', 'motor.yaml')

    return LaunchDescription([

        # IR scanner — 10 Hz, publishes /3A/scan in base_link frame
        Node(
            package='qupa_hardware',
            executable='ir_scanner',
            name='ir_scanner',
            namespace='3A',
            output='screen',
            parameters=[ir_cfg],
        ),

        # Motor driver — subscriber-driven on /3A/cmd_vel, watchdog at 20 Hz
        Node(
            package='qupa_hardware',
            executable='motor_driver',
            name='motor_node',
            namespace='3A',
            output='screen',
            parameters=[motor_cfg],
        ),

    ])
