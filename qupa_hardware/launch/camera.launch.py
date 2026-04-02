from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('qupa_hardware')
    cam_cfg = os.path.join(pkg, 'config', 'camera.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'calibration', default_value='false',
            description='Launch calibration node for RViz-based mask/HSV tuning'
        ),

        # Main camera — 5 Hz
        # Publishes: camera/image_filtered/compressed  (always, JPEG ~30-50 KB/frame)
        #            camera/image_raw                  (only when publish_raw: true)
        Node(
            package='qupa_hardware',
            executable='camera',
            name='camera_node',
            namespace='qupa_3A',
            output='screen',
            parameters=[
                cam_cfg,
                # Enable raw topic when calibration node is active
                {'publish_raw': LaunchConfiguration('calibration')},
            ],
        ),

        # Calibration — only launched when calibration:=true
        # Subscribes to image_raw (local loopback), publishes image_calibration/compressed
        # Tune live: ros2 param set /qupa_3A/camera_calibration_node inner_radius_px 70
        Node(
            package='qupa_hardware',
            executable='camera_calibration',
            name='camera_calibration_node',
            namespace='qupa_3A',
            output='screen',
            parameters=[cam_cfg],
            condition=IfCondition(LaunchConfiguration('calibration')),
        ),

    ])
