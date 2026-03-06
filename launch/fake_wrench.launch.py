from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake_wrench_script = PathJoinSubstitution(
        [FindPackageShare("mpc_controller"), "launch", "pub_fake_wrench.py"]
    )

    fake_wrench_publisher = ExecuteProcess(
        cmd=["python3", fake_wrench_script],
        output="screen",
    )

    return LaunchDescription([fake_wrench_publisher])
