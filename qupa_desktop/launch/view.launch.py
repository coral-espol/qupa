"""
view.launch.py — PC-side launch for QUPA visualisation.

Launches:
  - robot_state_publisher  (URDF / TF tree)
  - joint_state_publisher
  - rviz2                  (calibration.rviz config)

Arguments:
  calibration   true | false (default: false)
                When true, opens RViz with calibration.rviz which includes
                the camera/image_calibration/compressed display.

Usage:
  ros2 launch qupa_desktop view.launch.py
  ros2 launch qupa_desktop view.launch.py calibration:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg    = get_package_share_directory('qupa_description')
    desktop_pkg = get_package_share_directory('qupa_desktop')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'qupa_real.xacro')
    rviz_cfg   = os.path.join(desktop_pkg, 'config', 'calibration.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        ),

    ])
