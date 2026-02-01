from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        # required: coming from main launch
        DeclareLaunchArgument("taskFile", default_value=""),
        DeclareLaunchArgument("libFolder", default_value=""),
        DeclareLaunchArgument("urdfFile", default_value=""),
        DeclareLaunchArgument("globalFrame", default_value="odom"),
        DeclareLaunchArgument("robotName", default_value="mobile_manipulator"),

        # tt params
        DeclareLaunchArgument("publishRate", default_value="20.0"),
        DeclareLaunchArgument("monitorRate", default_value="50.0"),
        DeclareLaunchArgument("posTol", default_value="0.03"),
        DeclareLaunchArgument("oriTol", default_value="0.20"),
    ]

    tt_pub = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_tt_publisher_node",
        name="trajectory_tt_publisher",
        output="screen",
        parameters=[{
            "taskFile": LaunchConfiguration("taskFile"),
            "libFolder": LaunchConfiguration("libFolder"),
            "urdfFile": LaunchConfiguration("urdfFile"),
            "globalFrame": LaunchConfiguration("globalFrame"),
            "robotName": LaunchConfiguration("robotName"),
            "publishRate": LaunchConfiguration("publishRate"),
        }],
    )

    progress_monitor = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_progress_monitor_node",
        name="trajectory_progress_monitor",
        output="screen",
        parameters=[{
            "taskFile": LaunchConfiguration("taskFile"),
            "libFolder": LaunchConfiguration("libFolder"),
            "urdfFile": LaunchConfiguration("urdfFile"),

            "robotName": LaunchConfiguration("robotName"),
            "monitorRate": LaunchConfiguration("monitorRate"),
            "posTol": LaunchConfiguration("posTol"),
            "oriTol": LaunchConfiguration("oriTol"),
        }],
    )

    return LaunchDescription(declared_arguments + [tt_pub, progress_monitor])
