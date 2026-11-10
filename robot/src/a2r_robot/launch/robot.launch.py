from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    hardware = Node(package='a2r_robot',executable='hardware',namespace=namespace,
                    output='screen',parameters=[{
                        'uart':LaunchConfiguration('uart'),
                        'camera_device':LaunchConfiguration('camera_device'),
                        'motion_enabled':ParameterValue(LaunchConfiguration('motion_enabled'),value_type=bool),
                        'publish_raw_images':ParameterValue(LaunchConfiguration('publish_raw_images'),value_type=bool)}])
    safety = Node(package='a2r_robot',executable='safety',namespace=namespace,output='screen',
                  parameters=[{'track_width_m': ParameterValue(
                      LaunchConfiguration('track_width_m'), value_type=float)}])
    visualization = Node(package='a2r_robot',executable='visualization',namespace=namespace,
                         output='screen',condition=IfCondition(LaunchConfiguration('visualization')))
    return LaunchDescription([
        DeclareLaunchArgument('uart',default_value='/dev/ttyTHS1'),
        DeclareLaunchArgument('camera_device',default_value='/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_1080P_Low_Light_UC684-video-index0'),
        DeclareLaunchArgument('visualization',default_value='true'),
        DeclareLaunchArgument('namespace',default_value=EnvironmentVariable('ROBOT_NAMESPACE', default_value='jetson1')),
        DeclareLaunchArgument('motion_enabled',default_value='false'),
        DeclareLaunchArgument('track_width_m',default_value='0.0'),
        DeclareLaunchArgument('publish_raw_images',default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=hardware,
            on_exit=[EmitEvent(event=Shutdown(reason='Hardware node exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=safety,
            on_exit=[EmitEvent(event=Shutdown(reason='Safety node exited'))])),
        hardware,safety,visualization])
