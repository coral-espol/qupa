"""
vision.launch.py — PC-side metric vision for one robot.

Launches:
  target_ranger  — camera/detections (px) → camera/targets (m, rad) + RViz markers
  shape_observer — camera/targets → shape/detected (TRIANGLE/SQUARE/PENTAGON)
                   (disable with shapes:=false)

Loads config/vision_<namespace>.yaml (written by calibrate_range fit --out).

Usage:
  ros2 launch qupa_desktop vision.launch.py
  ros2 launch qupa_desktop vision.launch.py namespace:=qupa_3B
  ros2 launch qupa_desktop vision.launch.py namespace:=qupa_AE shapes:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    ns  = LaunchConfiguration('namespace').perform(context)
    cfg = os.path.join(get_package_share_directory('qupa_desktop'),
                       'config', f'vision_{ns}.yaml')

    actions = []
    params  = []
    if os.path.exists(cfg):
        params.append(cfg)
    else:
        actions.append(LogInfo(msg=f'[vision] {cfg} no existe — modelo sin calibrar'))

    actions.append(Node(
        package='qupa_desktop',
        executable='target_ranger',
        name='target_ranger',
        namespace=ns,
        output='screen',
        parameters=params,
    ))
    actions.append(Node(
        package='qupa_desktop',
        executable='shape_observer',
        name='shape_observer',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('shapes')),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='qupa_3A',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'shapes', default_value='true',
            description='Also run shape_observer (figure recognition)'
        ),
        OpaqueFunction(function=_setup),
    ])
