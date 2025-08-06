from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    motion_controller_pkg = get_package_share_directory('moveit_planner')
    calibration_pkg = get_package_share_directory('easy_handeye2')
    return LaunchDescription([

        # Declare toggle arguments
        DeclareLaunchArgument('run_realsense', default_value='true',
                              description='Run the RealSense camera node'),
        DeclareLaunchArgument('run_saver', default_value='true',
                              description='Run the berry saver node'),
        DeclareLaunchArgument('run_motion', default_value='false',
                              description='Run the UR trigger node'),


        # To include the simulation
        # IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(ur_description_pkg, 'launch', 'ur_sim_control.launch.py')
        #    ),
        #    launch_arguments={'ur_type': 'ur5e'}.items()
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(motion_controller_pkg, 'launch', 'moveit_planner.launch.py')
            )
        ),

        # RealSense streamer node
        Node(
            package='realsense_streamer',
            executable='RealSenseStreamer',
            name='realsense_streamer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_realsense'))
        ),
        # Camer tf publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(calibration_pkg, 'launch', 'publish.launch.py')
            )
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
