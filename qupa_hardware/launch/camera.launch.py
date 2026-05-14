"""
camera.launch.py — Normal operation, detection only (no image stream).

Launches:
  camera_node — publishes camera/detections (DetectionArray) only.
                No image topics → lower CPU and bandwidth.

Usage:
  ros2 launch qupa_hardware camera.launch.py
  ros2 launch qupa_hardware camera.launch.py namespace:=qupa_3B
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

    # Load YAML as a plain dict. Passing the file path directly relies on
    # ROS 2 matching the `camera_node:` key against the node's fully
    # qualified name, which fails silently when a namespace is applied.
    # Loading manually bypasses that matching entirely.
    with open(cam_cfg, 'r') as f:
        cam_params = yaml.safe_load(f)
    shared_params = cam_params.get('camera_node', {}).get('ros__parameters', {})

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace'
        ),

        Node(
            package='qupa_hardware',
            executable='camera',
            name='camera_node',
            namespace=ns,
            output='screen',
            parameters=[
                shared_params,
                {'publish_image': False, 'publish_raw': False},
            ],
        ),

    ])
