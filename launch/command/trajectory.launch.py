import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

common_dir = Path(__file__).resolve().parent.parent / "common"
if str(common_dir) not in sys.path:
    sys.path.insert(0, str(common_dir))

from config_utils import load_common_params, load_default_frame_from_robot_config  # noqa: E402


def generate_launch_description():
    params = load_common_params()
    trajectory_defaults = params.get("command", {}).get("trajectory", {})
    axis_defaults = trajectory_defaults.get("axis", {})
    base_trajectory_defaults = trajectory_defaults.get("base_trajectory", {})
    default_global_frame = load_default_frame_from_robot_config()

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        
        DeclareLaunchArgument("trajectoryPublishRate", default_value=str(trajectory_defaults.get("publish_rate", 100.0)), description="Trajectory target publish rate."),
        DeclareLaunchArgument("trajectoryHorizon", default_value=str(trajectory_defaults.get("horizon", 50.0)), description="Trajectory horizon (s)."),
        DeclareLaunchArgument("trajectoryDt", default_value=str(trajectory_defaults.get("hodtrizon", 0.01)), description="Trajectory discretization step (s)."),
        DeclareLaunchArgument("trajectoryAmplitude", default_value=str(trajectory_defaults.get("amplitude", 0.2)), description="Eight-shape amplitude (m)."),
        DeclareLaunchArgument("trajectoryFrequency", default_value=str(trajectory_defaults.get("frequency", 0.2)), description="Eight-shape frequency (Hz)."),
        DeclareLaunchArgument("baseTrajectoryAmplitude", default_value=str(base_trajectory_defaults.get("amplitude", trajectory_defaults.get("amplitude", 0.2))), description="Base eight-shape amplitude (m)."),
        DeclareLaunchArgument("baseTrajectoryFrequency", default_value=str(base_trajectory_defaults.get("frequency", trajectory_defaults.get("frequency", 0.2))), description="Base eight-shape frequency (Hz)."),
        DeclareLaunchArgument("trajectoryAxisX", default_value=str(axis_defaults.get("x", 1.0))),
        DeclareLaunchArgument("trajectoryAxisY", default_value=str(axis_defaults.get("y", 0.0))),
        DeclareLaunchArgument("trajectoryAxisZ", default_value=str(axis_defaults.get("z", 1.0))),
        DeclareLaunchArgument(
            "globalFrame",
            default_value=str(default_global_frame),
            description="Frame used for trajectory target markers.",
        ),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")

    trajectory_node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_trajectory_target",
        name="mobile_manipulator_trajectory_target",
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "publishRate": LaunchConfiguration("trajectoryPublishRate"),
            "horizon": LaunchConfiguration("trajectoryHorizon"),
            "dt": LaunchConfiguration("trajectoryDt"),
            "amplitude": LaunchConfiguration("trajectoryAmplitude"),
            "frequency": LaunchConfiguration("trajectoryFrequency"),
            "baseAmplitude": LaunchConfiguration("baseTrajectoryAmplitude"),
            "baseFrequency": LaunchConfiguration("baseTrajectoryFrequency"),
            "axisX": LaunchConfiguration("trajectoryAxisX"),
            "axisY": LaunchConfiguration("trajectoryAxisY"),
            "axisZ": LaunchConfiguration("trajectoryAxisZ"),
            "trajectoryGlobalFrame": LaunchConfiguration("globalFrame"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [trajectory_node])
