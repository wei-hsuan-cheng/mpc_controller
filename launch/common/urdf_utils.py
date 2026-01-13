import subprocess
from typing import Iterable, List, Optional


def generate_urdf_from_xacro(
    xacro_in: str,
    urdf_out: str,
    initial_pose: str,
    extra_args: Optional[Iterable[str]] = None,
) -> str:
    """Generate a URDF from the given xacro path using provided output/initial pose paths."""
    cmd: List[str] = [
        "xacro",
        xacro_in,
        f"initial_pose_file:={initial_pose}",
        "ros2_control_mode:=true",
    ]
    if extra_args:
        cmd.extend(list(extra_args))
    cmd.extend(["-o", urdf_out])

    try:
        subprocess.run(cmd, check=True)
        print(f"[mpc_controller] Generated URDF from xacro: {urdf_out}")
    except Exception as exc:  # noqa: BLE001
        print(f"[mpc_controller] Failed to generate URDF from xacro: {exc}")

    return urdf_out
