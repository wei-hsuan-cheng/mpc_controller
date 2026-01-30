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


def generate_launch_description():
    params = load_common_params()
    joint_cfg = params.get("command", {}).get("joint_trajectory", {})

    # Defaults from YAML
    publish_rate = float(joint_cfg.get("publish_rate", 100.0))
    horizon = float(joint_cfg.get("horizon", 5.0))
    dt = float(joint_cfg.get("dt", 0.01))
    amplitude = float(joint_cfg.get("amplitude", 0.05))
    frequency = float(joint_cfg.get("frequency", 0.2))
    nominal = joint_cfg.get("nominal_joint_positions", [])
    amp_per_joint = joint_cfg.get("amplitude_per_joint", [])
    phase_offsets = joint_cfg.get("phase_offsets", [])

    declared_arguments = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="mobile_manipulator",
            description="Robot name prefix for OCS2 topics.",
        ),
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF file path."),
    ]

    node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_joint_trajectory_target",
        name="mobile_manipulator_joint_trajectory_target",
        output="screen",
        parameters=[{
            # Topic prefix
            "robot_name": LaunchConfiguration("robot_name"),

            # OCS2 interface files
            "task_file": LaunchConfiguration("taskFile"),
            "lib_folder": LaunchConfiguration("libFolder"),
            "urdf_file": LaunchConfiguration("urdfFile"),

            # From YAML "joint_trajectory"
            "publish_rate_hz": publish_rate,
            "horizon_s": horizon,
            "dt_s": dt,

            "amplitude": amplitude,
            "frequency_hz": frequency,

            "q_nominal": nominal,
            "q_amplitude": amp_per_joint,
            "q_phase_rad": phase_offsets,
        }],
    )

    return LaunchDescription(declared_arguments + [node])
