"""Launch only the output-only collector; never starts the motion driver."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.2.40'),
        DeclareLaunchArgument('frequency', default_value='500.0'),
        DeclareLaunchArgument('queue_capacity', default_value='512'),
        DeclareLaunchArgument('expected_version', default_value='5.9.3'),
        DeclareLaunchArgument('timeout_s', default_value='3.0'),
        Node(package='ur_rtde_controller', executable='rtde_feedback_collector',
             name='rtde_feedback_collector', output='screen', parameters=[{
                 'robot_ip': LaunchConfiguration('robot_ip'),
                 'frequency': ParameterValue(LaunchConfiguration('frequency'), value_type=float),
                 'queue_capacity': ParameterValue(LaunchConfiguration('queue_capacity'), value_type=int),
                 'expected_version': LaunchConfiguration('expected_version'),
                 'timeout_s': ParameterValue(LaunchConfiguration('timeout_s'), value_type=float),
             }]),
    ])
