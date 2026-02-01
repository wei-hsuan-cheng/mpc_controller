import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value=""),
        DeclareLaunchArgument("libFolder", default_value=""),
        DeclareLaunchArgument("urdfFile", default_value=""),
        DeclareLaunchArgument("globalFrame", default_value="odom"),

        DeclareLaunchArgument("planner_rate", default_value="50.0"),
        DeclareLaunchArgument("frame_id", default_value=LaunchConfiguration("globalFrame")),
        DeclareLaunchArgument("publish_markers", default_value="true"),

        DeclareLaunchArgument("mpc_observation_topic", default_value="/mobile_manipulator_mpc_observation"),
        DeclareLaunchArgument("mpc_policy_topic", default_value="/mobile_manipulator_mpc_policy"),
        DeclareLaunchArgument("target_traj_topic", default_value="/mobile_manipulator_mpc_target_trajectories"),
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
            "globalFrame": LaunchConfiguration("frame_id"),
            "rate": LaunchConfiguration("planner_rate"),
            "publish_markers": LaunchConfiguration("publish_markers"),

            "mpc_observation_topic": LaunchConfiguration("mpc_observation_topic"),
            "mpc_policy_topic": LaunchConfiguration("mpc_policy_topic"),
            "target_traj_topic": LaunchConfiguration("target_traj_topic"),
        }],
    )

    progress_monitor = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_progress_monitor_node",
        name="trajectory_progress_monitor",
        output="screen",
        parameters=[{
            "globalFrame": LaunchConfiguration("frame_id"),
        }],
    )

    return LaunchDescription(declared_arguments + [tt_pub, progress_monitor])
