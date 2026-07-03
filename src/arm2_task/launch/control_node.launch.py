import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "arm2_task"
    pkg_share = get_package_share_directory(package_name)
    default_params_path = os.path.join(pkg_share, "config", "params.yaml")

    params_path_arg = DeclareLaunchArgument(
        "params_path",
        default_value=default_params_path,
        description="Full path to the arm2_task parameter file for control_node",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level for control_node",
    )

    control_node = Node(
        package=package_name,
        executable="control_node",
        name="controller_node",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_path")],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            params_path_arg,
            log_level_arg,
            control_node,
        ]
    )
