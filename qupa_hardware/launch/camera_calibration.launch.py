"""
camera_calibration.launch.py — Live stream for mask/HSV tuning in RViz.

Launches:
  camera_node             — publish_image:=true always, publish_raw only if raw:=true
  camera_calibration_node — overlay with rings, lines, bounding boxes

Usage:
  ros2 launch qupa_hardware camera_calibration.launch.py
  ros2 launch qupa_hardware camera_calibration.launch.py raw:=true
  ros2 launch qupa_hardware camera_calibration.launch.py namespace:=qupa_3B
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg     = get_package_share_directory('qupa_hardware')
    cam_cfg = os.path.join(pkg, 'config', 'camera.yaml')

    with open(cam_cfg, 'r') as f:
        shared_params = yaml.safe_load(f)['camera_node']['ros__parameters']

    ns  = LaunchConfiguration('namespace')
    raw = LaunchConfiguration('raw')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace'
        ),

        DeclareLaunchArgument(
            'raw', default_value='false',
            description='Also publish camera/image_raw (higher bandwidth)'
        ),

        Node(
            package='qupa_hardware',
            executable='camera',
            name='camera_node',
            namespace=ns,
            output='screen',
            parameters=[
                cam_cfg,
                {'publish_image': True, 'publish_raw': raw},
            ],
        ),

        Node(
            package='qupa_hardware',
            executable='camera_calibration',
            name='camera_calibration_node',
            namespace=ns,
            output='screen',
            parameters=[shared_params],
        ),

    ])
