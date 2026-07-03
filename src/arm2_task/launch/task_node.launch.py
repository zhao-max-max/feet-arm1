import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    manual_mode = mode == "terminal"

    task_node = Node(
        package="arm2_task",
        executable="task_node",
        name="task_manager_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_path"),
            {"task": {"manual_mode": manual_mode}},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return [task_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("arm2_task")
    default_params_path = os.path.join(pkg_share, "config", "task_params.yaml")

    params_path_arg = DeclareLaunchArgument(
        "params_path",
        default_value=default_params_path,
        description="Full path to the arm2_task parameter file for task_node",
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="nav",
        choices=["nav", "terminal"],
        description="Task interface mode: nav uses /arm/mission_event, terminal uses the interactive menu",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level for task_node",
    )

    return LaunchDescription(
        [
            params_path_arg,
            mode_arg,
            log_level_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
