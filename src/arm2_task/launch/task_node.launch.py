import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _load_task_parameters(params_path):
    if not os.path.exists(params_path):
        return {}
    with open(params_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("/**", {}).get("ros__parameters", {})


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _as_float_triplet(value, default):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return default
    return [float(value[0]), float(value[1]), float(value[2])]


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    manual_mode = mode == "terminal"
    params_path = LaunchConfiguration("params_path").perform(context)
    publish_lidar_tf = _as_bool(LaunchConfiguration("publish_lidar_tf").perform(context))
    ros_params = _load_task_parameters(params_path)

    lidar_tf_config = ros_params.get("task_nav", {}).get("lidar_extrinsics", {})
    lidar_tf_enabled = publish_lidar_tf and _as_bool(lidar_tf_config.get("enabled", True))
    lidar_parent_frame = str(lidar_tf_config.get("parent_frame", "base_link"))
    lidar_child_frame = str(lidar_tf_config.get("child_frame", "lidar_link"))
    lidar_translation = _as_float_triplet(lidar_tf_config.get("translation"), [0.127, 0.0, 0.0])
    lidar_rotation_rpy = _as_float_triplet(lidar_tf_config.get("rotation_rpy"), [0.0, 0.0, -1.570796])

    nodes = []

    if lidar_tf_enabled:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_lidar_link_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    str(lidar_translation[0]),
                    "--y",
                    str(lidar_translation[1]),
                    "--z",
                    str(lidar_translation[2]),
                    "--roll",
                    str(lidar_rotation_rpy[0]),
                    "--pitch",
                    str(lidar_rotation_rpy[1]),
                    "--yaw",
                    str(lidar_rotation_rpy[2]),
                    "--frame-id",
                    lidar_parent_frame,
                    "--child-frame-id",
                    lidar_child_frame,
                ],
            )
        )

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

    nodes.append(task_node)
    return nodes


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

    publish_lidar_tf_arg = DeclareLaunchArgument(
        "publish_lidar_tf",
        default_value="true",
        description="Publish task_nav.lidar_extrinsics as a static TF",
    )

    return LaunchDescription(
        [
            params_path_arg,
            mode_arg,
            log_level_arg,
            publish_lidar_tf_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
