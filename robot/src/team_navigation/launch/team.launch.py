from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('team_navigation')) /
                 'config' / 'navigation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Assigned robot namespace'),
        Node(package='team_navigation', executable='navigation',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[config]),
    ])
