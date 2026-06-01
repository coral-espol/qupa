"""
camera_calibration.launch.py — Live image stream for mask/HSV tuning in RViz.

Launches only camera_calibration_node — an independent node that opens its
own camera instance and publishes the annotated calibration image.
Does NOT launch camera_node.

Parameters are loaded from the camera_node section of camera.yaml so that
both nodes always share the same mask geometry and HSV values. Edit camera.yaml
once and both nodes pick up the change on the next launch.

Publishes:
  camera/image_calibration/compressed   (JPEG overlay for RViz)

Usage:
  ros2 launch qupa_hardware camera_calibration.launch.py
  ros2 launch qupa_hardware camera_calibration.launch.py namespace:=qupa_3B
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg     = get_package_share_directory('qupa_hardware')
    cam_cfg = os.path.join(pkg, 'config', 'camera.yaml')

    ns = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace'
        ),

        Node(
            package='qupa_hardware',
            executable='camera_calibration',
            name='camera_calibration_node',
            namespace=ns,
            output='screen',
            parameters=[cam_cfg],
        ),

    ])
