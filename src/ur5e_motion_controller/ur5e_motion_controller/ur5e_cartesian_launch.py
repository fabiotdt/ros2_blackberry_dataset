from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('ur5e_motion_controller/arm_description')

    urdf_file = os.path.join(pkg_path, 'urdf', 'ur5e.urdf.xacro')
    controller_yaml = os.path.join(pkg_path, 'config', 'controller.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controller_yaml
            ],
        ),

        Node(
            package='ur5e_motion_controller',
            executable='motion_data_loop',
            output='screen',
        ),
    ])