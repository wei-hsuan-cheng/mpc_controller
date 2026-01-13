import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

common_dir = Path(__file__).resolve().parent.parent / "common"
if str(common_dir) not in sys.path:
    sys.path.insert(0, str(common_dir))

from config_utils import load_common_params  # noqa: E402


def _bool_to_str(value):
    return "true" if value else "false"


def generate_launch_description():
    params = load_common_params()
    twist_defaults = params.get("command", {}).get("twist", {})
    vel_defaults = twist_defaults.get("velocity", {})
    omega_defaults = twist_defaults.get("omega", {})

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("twistPublishRate", default_value=str(twist_defaults.get("publish_rate", 20.0)), description="Twist target publish rate."),
        DeclareLaunchArgument("twistHorizon", default_value=str(twist_defaults.get("horizon", 1.0)), description="Twist target horizon (s)."),
        DeclareLaunchArgument("twistDt", default_value=str(twist_defaults.get("dt", 0.1)), description="Twist discretization step (s)."),
        DeclareLaunchArgument("twistInWorld", default_value=_bool_to_str(twist_defaults.get("twist_in_world", True)), description="Interpret twist in world frame."),
        DeclareLaunchArgument("twistVelX", default_value=str(vel_defaults.get("x", 0.0))),
        DeclareLaunchArgument("twistVelY", default_value=str(vel_defaults.get("y", 0.0))),
        DeclareLaunchArgument("twistVelZ", default_value=str(vel_defaults.get("z", 0.0))),
        DeclareLaunchArgument("twistOmegaX", default_value=str(omega_defaults.get("x", 0.0))),
        DeclareLaunchArgument("twistOmegaY", default_value=str(omega_defaults.get("y", 0.0))),
        DeclareLaunchArgument("twistOmegaZ", default_value=str(omega_defaults.get("z", 0.0))),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")

    twist_node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_twist_target",
        name="mobile_manipulator_twist_target",
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "publishRate": LaunchConfiguration("twistPublishRate"),
            "horizon": LaunchConfiguration("twistHorizon"),
            "dt": LaunchConfiguration("twistDt"),
            "twistInWorld": LaunchConfiguration("twistInWorld"),
            "vx": LaunchConfiguration("twistVelX"),
            "vy": LaunchConfiguration("twistVelY"),
            "vz": LaunchConfiguration("twistVelZ"),
            "wx": LaunchConfiguration("twistOmegaX"),
            "wy": LaunchConfiguration("twistOmegaY"),
            "wz": LaunchConfiguration("twistOmegaZ"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [twist_node])
