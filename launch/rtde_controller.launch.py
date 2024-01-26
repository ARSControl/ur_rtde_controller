from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def create_example_node():

    # Python Node - Parameters
    robot_parameters = {
        'ROBOT_IP':       LaunchConfiguration('ROBOT_IP'),
        'enable_gripper': LaunchConfiguration('enable_gripper'),
        'asynchronous':   LaunchConfiguration('asynchronous'),
        'limit_acc':      LaunchConfiguration('limit_acc'),
    }

    # Python Node + Parameters + YAML Config File
    example_node = Node(
        package='ur_rtde_controller', executable='rtde_controller', name='ur_rtde_controller',
        output='screen', emulate_tty=True, output_format='{line}', arguments=[('__log_level:=info')],
        parameters=[robot_parameters],
    )

    return example_node

def generate_launch_description():

    # Launch Description
    launch_description = LaunchDescription()

    # Robot Arguments
    ROBOT_IP_arg       = DeclareLaunchArgument('ROBOT_IP',       default_value='192.168.2.40')
    enable_gripper_arg = DeclareLaunchArgument('enable_gripper', default_value='true')
    asynchronous_arg   = DeclareLaunchArgument('asynchronous',   default_value='false')
    limit_acc_arg      = DeclareLaunchArgument('limit_acc',      default_value='true')

    # Launch Description - Add Arguments
    launch_description.add_action(ROBOT_IP_arg)
    launch_description.add_action(enable_gripper_arg)
    launch_description.add_action(asynchronous_arg)
    launch_description.add_action(limit_acc_arg)

    # Launch Description - Add Nodes
    launch_description.add_action(create_example_node())

    # Return Launch Description
    return launch_description
