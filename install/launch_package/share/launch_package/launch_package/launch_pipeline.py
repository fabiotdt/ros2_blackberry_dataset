from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Declare toggle arguments
        DeclareLaunchArgument('run_realsense', default_value='true',
                              description='Run the RealSense camera node'),
        DeclareLaunchArgument('run_saver', default_value='true',
                              description='Run the berry saver node'),
        DeclareLaunchArgument('run_trigger', default_value='true',
                              description='Run the UR trigger node'),

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

        # UR trigger publisher
        Node(
            package='ur_simulator',
            executable='ur_publisher',
            name='ur_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('run_trigger'))
        ),
    ])