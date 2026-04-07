from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('qupa_hardware')
    ir_cfg    = os.path.join(pkg, 'config', 'ir_scanner.yaml')
    motor_cfg = os.path.join(pkg, 'config', 'motor.yaml')
    floor_cfg = os.path.join(pkg, 'config', 'floor_sensor.yaml')
    leds_cfg  = os.path.join(pkg, 'config', 'leds.yaml')
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace — change per robot example qupa_3A'
        ),

        Node(
            package='qupa_hardware',
            executable='ir_scanner',
            name='ir_scanner',
            namespace=ns,
            output='screen',
            parameters=[ir_cfg],
        ),

        Node(
            package='qupa_hardware',
            executable='motor_driver',
            name='motor_node',
            namespace=ns,
            output='screen',
            parameters=[motor_cfg],
        ),

        Node(
            package='qupa_hardware',
            executable='floor_sensor',
            name='floor_sensor_node',
            namespace=ns,
            output='screen',
            parameters=[floor_cfg],
        ),

        Node(
            package='qupa_hardware',
            executable='led',
            name='led_node',
            namespace=ns,
            output='screen',
            parameters=[leds_cfg],
        ),

    ])
