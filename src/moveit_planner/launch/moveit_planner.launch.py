from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Timer action to delay the execution of the launch description
from launch.actions import TimerAction


def generate_launch_description():
    ur_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5e_motion_controller"),
                    "launch",
                    "ur5e_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": "192.168.100.14",
            "launch_rviz": "false",
            "headless_mode": "true",
        }.items(),
    )
    moveit_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5e_berry_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
    )
    # Delay the execution of the moveit_planner launch file
    delay_moveit_planner = TimerAction(
        period=5.0,  # Delay for 5 seconds
        actions=[moveit_planner],
    )
    # collision loader
    collision_loader = Node(
        package="moveit_planner",
        executable="collision_loader_node",
        name="collision_loader_node",
        output="screen",
    )
    delayed_collision_loader = TimerAction(
        period=6.0,  # Delay for 5 seconds
        actions=[collision_loader],
    )
    # # moveit_commander_node
    moveit_commander = Node(
        package="moveit_planner",
        executable="moveit_commander_node",
        name="moveit_commander_node",
        output="screen",
    )
    delayed_moveit_commander = TimerAction(
        period=7.0,  # Delay for 7 seconds
        actions=[moveit_commander],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("moveit_planner"),
                    "rviz",
                    "moveit_rviz.rviz",
                ]
            ),
        ],
    )
    delayed_rviz_node = TimerAction(
        period=6.0,  # Delay for 8 seconds
        actions=[rviz_node],
    )
    return LaunchDescription(
        [
            ur_controller,
            delay_moveit_planner,
            delayed_rviz_node,
            delayed_collision_loader,
            delayed_moveit_commander,
        ]
    )
