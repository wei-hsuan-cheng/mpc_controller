import os
import yaml

from ament_index_python.packages import get_package_share_directory


def load_common_params(package_name: str = "mpc_controller", filename: str = "common_params.yaml"):
    """Load shared command/control parameters from the package config."""
    params_path = os.path.join(get_package_share_directory(package_name), "config", filename)
    with open(params_path, "r") as f:
        return yaml.safe_load(f)


def load_default_frame_from_robot_config(
    package_name: str = "mpc_controller",
    config_rel_path: str = os.path.join("config", "ridgeback_ur5", "ridgeback_ur5_ros2_control.yaml"),
    controller_key: str = "ridgeback_base_controller",
    default_frame: str = "odom",
):
    """Pull the odom/global frame from the robot's ros2_control config."""
    cfg_path = os.path.join(get_package_share_directory(package_name), config_rel_path)
    try:
        with open(cfg_path, "r") as f:
            diff_cfg = yaml.safe_load(f) or {}
        return (
            diff_cfg.get(controller_key, {})
            .get("ros__parameters", {})
            .get("odom_frame_id", default_frame)
        )
    except Exception:
        return default_frame
