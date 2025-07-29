from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Whether to use fake hardware or real hardware",
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # UR controller node (first to launch)
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
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    # MoveIt planner after 5s
    moveit_planner = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur5e_berry_moveit_config"),
                            "launch",
                            "move_group.launch.py",
                        ]
                    )
                )
            )
        ],
    )

    # RViz node after 6s
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=[
            #         "-d",
            #         PathJoinSubstitution(
            #             [
            #                 FindPackageShare("moveit_planner"),
            #                 "rviz",
            #                 "moveit_rviz.rviz",
            #             ]
            #         ),
            #     ],
            # )
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur5e_berry_moveit_config"),
                            "launch",
                            "moveit_rviz.launch.py",
                        ]
                    )
                ),
            )
        ],
    )

    # Collision loader after 6s
    collision_loader = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="moveit_planner",
                executable="collision_loader_node",
                name="collision_loader_node",
                output="screen",
            )
        ],
    )

    # MoveIt commander after 7s
    moveit_commander = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="moveit_planner",
                executable="moveit_commander_node",
                name="moveit_commander_node",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            use_fake_hardware_arg,
            ur_controller,
            moveit_planner,
            rviz_node,
            collision_loader,
            # moveit_commander,
        ]
    )
