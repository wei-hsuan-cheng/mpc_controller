from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tt_params = LaunchConfiguration('tt_params')
    
    declared_arguments = [
        DeclareLaunchArgument('robotName', default_value='mobile_manipulator'),
        DeclareLaunchArgument('tt_params', default_value=os.path.join(
            get_package_share_directory('mpc_cartesian_planner'), 'config', 'tt_params.yaml')),
        
        DeclareLaunchArgument("taskFile", default_value=""),
        DeclareLaunchArgument("libFolder", default_value=""),
        DeclareLaunchArgument("urdfFile", default_value=""),
        DeclareLaunchArgument("globalFrame", default_value=""),
        
    ]

    tt_pub = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_tt_publisher_node",
        name="trajectory_tt_publisher",
        output="screen",
        parameters=[tt_params,
                    {
                     "robotName": LaunchConfiguration("robotName"),
                     "taskFile": LaunchConfiguration("taskFile"),
                     "libFolder": LaunchConfiguration("libFolder"),
                     "urdfFile": LaunchConfiguration("urdfFile"),
                     "trajectoryGlobalFrame": LaunchConfiguration("globalFrame"),
                    },
                    ],
    )

    progress_monitor = Node(
        package="mpc_cartesian_planner",
        executable="trajectory_progress_monitor_node",
        name="trajectory_progress_monitor",
        output="screen",
        parameters=[tt_params,
                    {
                     "robotName": LaunchConfiguration("robotName"),
                     "taskFile": LaunchConfiguration("taskFile"),
                     "libFolder": LaunchConfiguration("libFolder"),
                     "urdfFile": LaunchConfiguration("urdfFile"),
                     "trajectoryGlobalFrame": LaunchConfiguration("globalFrame"),
                    },
                    ],
    )

    return LaunchDescription(declared_arguments + [tt_pub, progress_monitor])
