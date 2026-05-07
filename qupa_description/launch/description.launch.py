from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('qupa_description')
    xacro_file = os.path.join(pkg, 'urdf', 'qupa_real.xacro')

    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = Command(['xacro ', xacro_file, ' tf_prefix:=', ns])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'namespace', default_value='qupa',
            description='Robot namespace and TF prefix (e.g. qupa_3A)'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=ns,
            output='screen',
        ),
    ])
