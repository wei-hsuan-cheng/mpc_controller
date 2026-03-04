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
    joint_defaults = params.get("command", {}).get("joint", {})

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("jointPublishRate", default_value=str(joint_defaults.get("publish_rate", 20.0)),
                              description="Joint target publish rate."),
        DeclareLaunchArgument("jointHorizon", default_value=str(joint_defaults.get("horizon", 2.0)),
                              description="Joint target horizon (s)."),
        DeclareLaunchArgument("jointDt", default_value=str(joint_defaults.get("dt", 0.05)),
                              description="Joint target discretization step (s)."),
        DeclareLaunchArgument("jointRelativeToCurrent",
                              default_value=_bool_to_str(joint_defaults.get("relative_to_current", True)),
                              description="Use offsets relative to the arm state captured at the first observation."),
        DeclareLaunchArgument("jointSinusoidEnabled",
                              default_value=_bool_to_str(joint_defaults.get("sinusoid_enabled", True)),
                              description="Add a sinusoidal offset to each joint target."),
        DeclareLaunchArgument("jointSinusoidAmplitude",
                              default_value=str(joint_defaults.get("sinusoid_amplitude", 0.03)),
                              description="Sinusoidal joint offset amplitude [rad]."),
        DeclareLaunchArgument("jointSinusoidFrequency",
                              default_value=str(joint_defaults.get("sinusoid_frequency", 0.10)),
                              description="Sinusoidal joint offset frequency [Hz]."),
        DeclareLaunchArgument("jointSinusoidPhaseIncrement",
                              default_value=str(joint_defaults.get("sinusoid_phase_increment", 0.35)),
                              description="Per-joint phase increment [rad]."),
    ]

    joint_node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_joint_target",
        name="mobile_manipulator_joint_target",
        parameters=[{
            "taskFile": LaunchConfiguration("taskFile"),
            "libFolder": LaunchConfiguration("libFolder"),
            "urdfFile": LaunchConfiguration("urdfFile"),
            "publishRate": LaunchConfiguration("jointPublishRate"),
            "horizon": LaunchConfiguration("jointHorizon"),
            "dt": LaunchConfiguration("jointDt"),
            "relativeToCurrent": LaunchConfiguration("jointRelativeToCurrent"),
            "sinusoidEnabled": LaunchConfiguration("jointSinusoidEnabled"),
            "sinusoidAmplitude": LaunchConfiguration("jointSinusoidAmplitude"),
            "sinusoidFrequency": LaunchConfiguration("jointSinusoidFrequency"),
            "sinusoidPhaseIncrement": LaunchConfiguration("jointSinusoidPhaseIncrement"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [joint_node])
