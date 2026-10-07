"""
vision.launch.py — PC-side metric vision for one robot.

Launches:
  target_ranger  — camera/detections (px) → camera/targets (m, rad) + RViz markers
  shape_observer — camera/targets → shape/detected (TRIANGLE/SQUARE/PENTAGON)
                   (disable with shapes:=false)
  rviz2          — live top-down view of the targets (enable with rviz:=true)

Loads config/vision_<namespace>.yaml (written by calibrate_range fit --out).

Usage:
  ros2 launch qupa_desktop vision.launch.py
  ros2 launch qupa_desktop vision.launch.py namespace:=qupa_3B
  ros2 launch qupa_desktop vision.launch.py namespace:=qupa_AE shapes:=false
  ros2 launch qupa_desktop vision.launch.py namespace:=qupa_AE rviz:=true
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    ns  = LaunchConfiguration('namespace').perform(context)
    share = get_package_share_directory('qupa_desktop')
    cfg = os.path.join(share, 'config', f'vision_{ns}.yaml')

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

    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1'):
        # Frame and topic names depend on the namespace → fill in the template
        with open(os.path.join(share, 'config', 'vision.rviz.template')) as f:
            rviz_cfg = f.read().replace('@NS@', ns)
        rviz_path = os.path.join(tempfile.gettempdir(), f'qupa_vision_{ns}.rviz')
        with open(rviz_path, 'w') as f:
            f.write(rviz_cfg)
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_vision',
            arguments=['-d', rviz_path],
            output='log',
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
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Open RViz with a live top-down view of the targets'
        ),
        OpaqueFunction(function=_setup),
    ])
