from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ur_description_pkg = get_package_share_directory('ur_description')
    motion_controller_pkg = get_package_share_directory('ur5e_motion_controller')
    ur_simulation_pkg = get_package_share_directory('ur_simulation_gz')

    return LaunchDescription([

        # Declare toggle arguments
        DeclareLaunchArgument('run_realsense', default_value='true',
                              description='Run the RealSense camera node'),
        DeclareLaunchArgument('run_saver', default_value='true',
                              description='Run the berry saver node'),
        DeclareLaunchArgument('run_motion', default_value='false',
                              description='Run the UR trigger node'),


        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(ur_description_pkg, 'launch', 'ur_sim_control.launch.py')
           ),
           launch_arguments={'ur_type': 'ur5e'}.items()
        ),


        # State publisher node (your own)
        Node(
            package='ur5e_motion_controller',
            executable='arm_state_publisher',
            name='arm_state_publisher',
            output='screen',
        ),

        # motion_executor node with keyboard input support
        Node(
            package='ur5e_motion_controller',
            executable='motion_executor',
            name='motion_executor',
            output='screen',
            emulate_tty=True,  # enables keyboard input. Not working.
            condition=IfCondition(LaunchConfiguration('run_motion'))
        ),

        # RealSense streamer node
        Node(
            package='realsense_streamer',
            executable='RealSenseStreamer',
            name='realsense_streamer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_realsense'))
        ),

        # Berry saver node
        Node(
            package='berry_dataset',
            executable='dataset_saver',
            name='berry_saver',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_saver'))
        ),

    ])
