import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable,
    Shutdown
)
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    # Arm hand
    robot_ip = LaunchConfiguration('robot_ip')
    controllers_config = PathJoinSubstitution(
        [FindPackageShare('robot_launcher'), "config", "ros2_controllers.yaml"]
    )
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare('robot_launcher'), "config/moveit", "kinematics.yaml"]
    )
    # Initialize Arguments
    mobile_type = LaunchConfiguration("mobile_type")
    world_file = LaunchConfiguration("world_file")
    arm_prefix = LaunchConfiguration("arm_prefix")
    mobile_prefix = LaunchConfiguration("mobile_prefix")
    pose_xyz = LaunchConfiguration("pose_xyz")
    pose_rpy = LaunchConfiguration("pose_rpy")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_isaac = LaunchConfiguration("sim_isaac")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    namespace = LaunchConfiguration("namespace")
    description_file = LaunchConfiguration("description_file")
    base_robot_type = LaunchConfiguration("base_robot_type")
    
    # For OCS2 mpc param files
    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    ocs2_urdf_file = LaunchConfiguration("Ocs2UrdfFile")
    debug_flag = LaunchConfiguration("debug")
    global_frame = LaunchConfiguration("globalFrame")
    command_type = LaunchConfiguration("commandType")

    use_sim_time = True
    use_fake_hardware_bool = use_fake_hardware.perform(context) == "true" or use_fake_hardware.perform(context) == "True"


    cp = ParameterFile(controllers_config, allow_substs=True)
    with open(cp.evaluate(context), 'r') as f, open(
    '/tmp/launch____controllers_config.yaml', 'w') as h:
        h.write(f.read())

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_launcher"), "urdf", description_file]
            ),
            " ",
            "arm_prefix:=",
            arm_prefix,
            " ",
            "gripper_prefix:=",
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "esm_dt:=",
            "20",
            " ",
            "name:=",
            "mobile_base",
            " ",
            "robot_type:=",
            mobile_type,
            " ",
            "mobile_prefix:=",
            mobile_prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_isaac:=",
            sim_isaac,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "simulation_controllers:=",
            '/tmp/launch____controllers_config.yaml',
            " ",
            "controller_configs:=",
            controllers_config,
            " ",
            "kinematics_yaml:=",
            kinematics_yaml,
            " ",
            "base_robot_type:=",
            base_robot_type,
            " ",
        ]
    )
    robot_description = {'robot_description': ParameterValue(
        robot_description_content, value_type=str)}
    # planning_contex
    robot_description_semantic_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('robot_launcher'), 'config/moveit', 'rsm70.srdf.xacro']
            ),
            " ",
            "arm_prefix:=",
            arm_prefix,
            " ",
            "gripper_prefix:=",
            " ",
            "mobile_prefix:=",
            mobile_prefix,
        ]
    )
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'robot_launcher', 'config/moveit/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_controllers = load_yaml(
        'robot_launcher', 'config/moveit/moveit_controllers.yaml'
    )

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
        ],
    )
    # RViz
    rvizconfig = PathJoinSubstitution(
        [FindPackageShare('robot_launcher'), "config/rviz", "ocs2_mobile_manipulator.rviz"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rvizconfig],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Publish TF
    robot_description_publisher = Node(
        package='delta_arm_controllers',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='both',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time},
        ],
        # namespace='/ffaa',
        remappings = [
            ('~/robot_description', '/robot_description'),
            ('~/robot_description_semantic', '/robot_description_semantic'),
            ('~/describe_parameters', 'robot_state_publisher/describe_parameters'),
            ('~/get_parameter_types', 'robot_state_publisher/get_parameter_types'),
            ('~/get_parameters', 'robot_state_publisher/get_parameters'),
            ('~/list_parameters', 'robot_state_publisher/list_parameters'),
            ('~/set_parameters', 'robot_state_publisher/set_parameters'),
            ('~/set_parameters_atomically', 'robot_state_publisher/set_parameters_atomically'),
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            robot_description,    # From xacro command
            ParameterFile(controllers_config, allow_substs=True),
            kinematics_yaml,
            {"use_sim_time": use_sim_time},
            {
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": ocs2_urdf_file,
            }
        ],
        remappings=[('~/robot_description', '/robot_description')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )
    # Load controllers
    load_controllers = []
    for controller in [
        'robot_state_broadcaster',
        'delta_gripper_action_controller',
        'mb_servo_controller',
        'mpc_controller',
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]


    # Spawn robot
    pose_xyz_text = pose_xyz.perform(context).split(' ')
    pose_rpy_text = pose_rpy.perform(context).split(' ')
    if len(pose_xyz_text) != 3: pose_xyz_text = ['0.0', '0.0', '0.0']
    if len(pose_rpy_text) != 3: pose_rpy_text = ['0.0', '0.0', '0.0']

    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mobile_camera',
        arguments=['0.0', '0', '0.0', '0', '0', '0', '/mobile_mf_camera_link', '/camera_link'],
        output='screen'
    )
    
    map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )
    
    static_tf_nodes = [camera_tf_node, map_tf_node]

    load_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=
                load_controllers,
        )
    )
    
    mpc_node = Node(
        package="mpc_controller",
        executable="mobile_manipulator_mpc_node",
        name="mobile_manipulator_mpc",
        condition=UnlessCondition(debug_flag),
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": ocs2_urdf_file,
        }],
        output="screen",
    )
    
    # Command interface launcher (marker / twist / trajectory)
    command_dir = PathJoinSubstitution(
        [FindPackageShare("mpc_controller"), "launch", "command"]
    )
    command_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                command_dir,
                PythonExpression(["'", command_type, "'", " + '.launch.py'"]),
            ])
        ),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": ocs2_urdf_file,
            "globalFrame": global_frame,
        }.items(),
    )
    
    nodes_to_start = [
        load_controller,
        rviz_node,
        robot_description_publisher,
        run_move_group_node,
        control_node,
        mpc_node,
        command_include,
    ]
    
    nodes_to_start += static_tf_nodes

    return nodes_to_start


def generate_launch_description():
    mpc_package_share = FindPackageShare("mpc_controller")
    task_default = PathJoinSubstitution([
        mpc_package_share,
        "config",
        "robot",
        "task.info",
    ])

    lib_default = PathJoinSubstitution([
        mpc_package_share,
        "auto_generated",
        "robot",
    ])

    # Use pre-generated static URDF with fake hardware and velocity interfaces for arm joints.
    ocs2_urdf_default = PathJoinSubstitution([
        mpc_package_share,
        "description",
        "robot",
        "urdf",
        "robot.static.urdf",
    ])
    
    initial_pose_default = PathJoinSubstitution([
        mpc_package_share,
        "config",
        "robot",
        "initial_pose.yaml",
    ])

    
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="two_walls.world",
            description="world description file for gazebo simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mobile_prefix",
            default_value="mobile_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="arm prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mobile_type",
            default_value="delta_mb",
            description="mobile base type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_xyz",
            default_value="0.0 0.0 0.0",
            description="xyz value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_rpy",
            default_value="0.0 0.0 0.0",
            description="rpy value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",
            description="Using gazebo or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Using isaac sim or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Using fake hardware or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for node"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='Hostname or IP address of the robot.')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "base_robot_type",
            default_value="delta_mb",
            description="mobile base type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "taskFile",
            default_value=task_default,
            description="Path to the OCS2 task file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "libFolder",
            default_value=lib_default,
            description="Folder for auto-generated OCS2 libraries.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "Ocs2UrdfFile",
            default_value=ocs2_urdf_default,
            description="URDF passed to visualization nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Skip MPC node when true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "commandType",
            default_value="marker",
            description="Command interface to start: marker, twist, or trajectory.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "globalFrame",
            default_value="odom",
            description="Frame used for the interactive marker.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
