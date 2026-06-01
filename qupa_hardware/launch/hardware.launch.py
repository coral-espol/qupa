import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('qupa_hardware')

    def _load(file_key, yaml_path):
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)
        return cfg.get(file_key, {}).get('ros__parameters', {})

    ir_params    = _load('ir_scanner',      os.path.join(pkg, 'config', 'ir_scanner.yaml'))
    motor_params = _load('motor_node',      os.path.join(pkg, 'config', 'motor.yaml'))
    floor_params = _load('floor_sensor_node', os.path.join(pkg, 'config', 'floor_sensor.yaml'))
    leds_params  = _load('led_node',        os.path.join(pkg, 'config', 'leds.yaml'))

    ns = LaunchConfiguration('namespace')

    ir_node = Node(
        package='qupa_hardware',
        executable='ir_scanner',
        name='ir_scanner',
        namespace=ns,
        output='screen',
        parameters=[ir_params],
    )

    motor_node = Node(
        package='qupa_hardware',
        executable='motor_driver',
        name='motor_node',
        namespace=ns,
        output='screen',
        parameters=[motor_params],
    )

    floor_node = Node(
        package='qupa_hardware',
        executable='floor_sensor',
        name='floor_sensor_node',
        namespace=ns,
        output='screen',
        parameters=[floor_params],
    )

    led_node = Node(
        package='qupa_hardware',
        executable='led',
        name='led_node',
        namespace=ns,
        output='screen',
        parameters=[leds_params],
    )

    uv_led_node = Node(
        package='qupa_hardware',
        executable='uv_led',
        name='uv_led',
        namespace=ns,
        output='screen',
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace — change per robot (e.g. qupa_3B)'
        ),

        ir_node,
        TimerAction(period=3.0,  actions=[motor_node]),
        TimerAction(period=6.0,  actions=[floor_node]),
        TimerAction(period=9.0,  actions=[led_node]),
        TimerAction(period=10.0, actions=[uv_led_node]),

    ])