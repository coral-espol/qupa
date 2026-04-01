from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('qupa_hardware'),
        'config',
        'ir_scanner.yaml'
    )

    return LaunchDescription([
        Node(
            package='qupa_hardware',
            executable='ir_scanner',
            name='ir_scanner',
            namespace='3A',
            output='screen',
            parameters=[config],
        )
    ])
