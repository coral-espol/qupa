import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg     = get_package_share_directory('qupa_hardware')
    cam_cfg = os.path.join(pkg, 'config', 'camera.yaml')

    with open(cam_cfg, 'r') as f:
        shared_params = yaml.safe_load(f)['camera_node']['ros__parameters']

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace — change per robot (e.g. qupa_3B)'
        ),

        DeclareLaunchArgument(
            'calibration', default_value='false',
            description='Launch calibration node for RViz-based mask/HSV tuning'
        ),

        Node(
            package='qupa_hardware',
            executable='camera',
            name='camera_node',
            namespace=ns,
            output='screen',
            parameters=[
                cam_cfg,
                {'publish_raw': LaunchConfiguration('calibration')},
            ],
        ),

        Node(
            package='qupa_hardware',
            executable='camera_calibration',
            name='camera_calibration_node',
            namespace=ns,
            output='screen',
            parameters=[shared_params],
            condition=IfCondition(LaunchConfiguration('calibration')),
        ),

    ])
