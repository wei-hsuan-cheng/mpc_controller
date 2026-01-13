import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mpc_share = FindPackageShare("mpc_controller")

    task_default = PathJoinSubstitution([mpc_share, "config", "ridgeback_ur5", "task.info"])
    urdf_default = PathJoinSubstitution([mpc_share, "description", "ridgeback_ur5", "urdf", "ridgeback_ur5.urdf"])
    controllers_default = PathJoinSubstitution(
        [mpc_share, "config", "ridgeback_ur5", "ros2_controllers.yaml"]
    )
    arm_control_xacro = PathJoinSubstitution(
        [mpc_share, "description", "ridgeback_ur5", "urdf", "ur5_arm_ros2_control.urdf.xacro"]
    )

    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("use_fake_odom", default_value="true"),
        DeclareLaunchArgument("taskFile", default_value=task_default),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default),
        DeclareLaunchArgument("libFolder", default_value="/tmp/ocs2_auto_generated/ridgeback_ur5"),
        DeclareLaunchArgument("controllersFile", default_value=controllers_default),
        DeclareLaunchArgument("globalFrame", default_value="odom"),
        DeclareLaunchArgument("baseCmdTopic", default_value="/cmd_vel"),
        DeclareLaunchArgument("odomTopic", default_value="/odom"),
        DeclareLaunchArgument("markerPublishRate", default_value="100.0"),
        DeclareLaunchArgument("enableJoystick", default_value="false"),
        DeclareLaunchArgument("commandType", default_value="marker"),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            arm_control_xacro,
            " ",
            "use_mock_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
        ],
    )

    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_mpc_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "mpc_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_on_control_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[spawn_joint_state_broadcaster, spawn_mpc_controller],
        )
    )

    mpc_node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_mpc_node",
        name="mobile_manipulator_mpc",
        output="screen",
        parameters=[
            {
                "taskFile": LaunchConfiguration("taskFile"),
                "urdfFile": LaunchConfiguration("urdfFile"),
                "libFolder": LaunchConfiguration("libFolder"),
            }
        ],
    )

    fake_odom_node = Node(
        package="mpc_controller",
        executable="fake_base_odom_node",
        name="fake_base_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_fake_odom")),
        parameters=[
            {
                "cmd_vel_topic": LaunchConfiguration("baseCmdTopic"),
                "odom_topic": LaunchConfiguration("odomTopic"),
                "frame_id": LaunchConfiguration("globalFrame"),
                "child_frame_id": "base_link",
                "publish_rate": 200.0,
            }
        ],
    )

    # Command interface launcher (marker / twist / trajectory)
    command_dir = PathJoinSubstitution(
        [FindPackageShare("mpc_controller"), "launch", "command"]
    )
    command_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                command_dir,
                PythonExpression(["'", command_type, "'", " + '.launch.py'"]),
            ])
        ),
        launch_arguments={
            "taskFile": LaunchConfiguration("taskFile"),
            "libFolder": LaunchConfiguration("libFolder"),
            "urdfFile": LaunchConfiguration("urdfFile"),
            "globalFrame": LaunchConfiguration("globalFrame"),
        }.items(),
    )

    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mpc_share, "launch", "visualization", "visualize.launch.py"])
        ),
        launch_arguments={
            "urdfFile": LaunchConfiguration("urdfFile"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            spawn_on_control_start,
            fake_odom_node,
            mpc_node,
            command_launch,
            visualize_launch,
        ]
    )
