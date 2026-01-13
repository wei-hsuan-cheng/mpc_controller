import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ocs2_share = get_package_share_directory("ocs2_ros2_control")
    rviz_default = f"{ocs2_share}/rviz/mobile_manipulator.rviz"

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name="urdfFile",
        ),
        launch.actions.DeclareLaunchArgument(
            name="test",
            default_value="false",
        ),
        launch.actions.DeclareLaunchArgument(
            name="rviz",
            default_value="true",
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=rviz_default,
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[LaunchConfiguration("urdfFile")],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[LaunchConfiguration("urdfFile")],
            condition=IfCondition(LaunchConfiguration("test")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mobile_manipulator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        ),
    ])


if __name__ == "__main__":
    generate_launch_description()
