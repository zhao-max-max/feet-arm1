from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_state_topic_arg = DeclareLaunchArgument(
        "nav_state_topic",
        default_value="/navigation/state",
        description="Mock odometry topic consumed by arm nav_pose_tracker",
    )
    nav_task_points_topic_arg = DeclareLaunchArgument(
        "nav_task_points_topic",
        default_value="/navigation/task_points",
        description="Mock task-point topic consumed by arm nav_pose_tracker",
    )
    arm_mission_service_arg = DeclareLaunchArgument(
        "arm_mission_service",
        default_value="/arm/mission_event",
        description="Mission service provided by arm task_node",
    )
    arm_debug_state_service_arg = DeclareLaunchArgument(
        "arm_debug_state_service",
        default_value="/arm/debug_state_command",
        description="Debug state service provided by arm task_node",
    )
    nav_arm_event_service_arg = DeclareLaunchArgument(
        "nav_arm_event_service",
        default_value="/navigation/arm_event",
        description="Service served by this UI for arm callbacks",
    )
    nav_mission_request_topic_arg = DeclareLaunchArgument(
        "nav_mission_request_topic",
        default_value="/debug/nav/mission_request",
        description="Mirrored MissionCommand request debug topic",
    )
    nav_mission_response_topic_arg = DeclareLaunchArgument(
        "nav_mission_response_topic",
        default_value="/debug/nav/mission_response",
        description="Mirrored MissionCommand response debug topic",
    )
    nav_arm_event_request_topic_arg = DeclareLaunchArgument(
        "nav_arm_event_request_topic",
        default_value="/debug/nav/arm_event_request",
        description="Mirrored arm_event request debug topic",
    )
    nav_arm_event_response_topic_arg = DeclareLaunchArgument(
        "nav_arm_event_response_topic",
        default_value="/debug/nav/arm_event_response",
        description="Mirrored arm_event response debug topic",
    )
    pick_service_name_arg = DeclareLaunchArgument(
        "pick_service_name",
        default_value="get_pick_pos",
        description="Mock pick service provided by this UI",
    )
    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic",
        default_value="/task/target_pose",
        description="Arm target-pose debug topic to overlay on the map",
    )
    pose_auto_publish_arg = DeclareLaunchArgument(
        "pose_auto_publish",
        default_value="true",
        description="Whether to auto-publish the mock lidar pose",
    )
    mock_nav_publish_enabled_arg = DeclareLaunchArgument(
        "mock_nav_publish_enabled",
        default_value="true",
        description="Whether this UI publishes mock /navigation/state and /navigation/task_points",
    )
    pose_publish_hz_arg = DeclareLaunchArgument(
        "pose_publish_hz",
        default_value="10.0",
        description="Mock lidar pose publish rate",
    )
    vision_override_enabled_arg = DeclareLaunchArgument(
        "vision_override_enabled",
        default_value="false",
        description="Whether nav_mock_ui should override get_pick_pos",
    )
    vision_pick_z_arg = DeclareLaunchArgument(
        "vision_pick_z",
        default_value="0.12",
        description="Returned pick z in arm world coordinates",
    )
    vision_lidar_in_arm_x_arg = DeclareLaunchArgument(
        "vision_lidar_in_arm_x",
        default_value="0.127",
        description="Lidar x offset in arm base frame used by mock pick conversion",
    )
    vision_lidar_in_arm_y_arg = DeclareLaunchArgument(
        "vision_lidar_in_arm_y",
        default_value="0.0",
        description="Lidar y offset in arm base frame used by mock pick conversion",
    )
    vision_lidar_in_arm_yaw_rad_arg = DeclareLaunchArgument(
        "vision_lidar_in_arm_yaw_rad",
        default_value="-1.570796",
        description="Lidar yaw offset in arm base frame used by mock pick conversion",
    )

    nav_mock_ui = Node(
        package="debug_tool",
        executable="nav_mock_ui",
        name="nav_mock_ui_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "nav_state_topic": LaunchConfiguration("nav_state_topic"),
                "nav_task_points_topic": LaunchConfiguration("nav_task_points_topic"),
                "arm_mission_service": LaunchConfiguration("arm_mission_service"),
                "arm_debug_state_service": LaunchConfiguration("arm_debug_state_service"),
                "nav_arm_event_service": LaunchConfiguration("nav_arm_event_service"),
                "nav_mission_request_topic": LaunchConfiguration("nav_mission_request_topic"),
                "nav_mission_response_topic": LaunchConfiguration("nav_mission_response_topic"),
                "nav_arm_event_request_topic": LaunchConfiguration("nav_arm_event_request_topic"),
                "nav_arm_event_response_topic": LaunchConfiguration("nav_arm_event_response_topic"),
                "pick_service_name": LaunchConfiguration("pick_service_name"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "pose_auto_publish": LaunchConfiguration("pose_auto_publish"),
                "mock_nav_publish_enabled": LaunchConfiguration("mock_nav_publish_enabled"),
                "pose_publish_hz": LaunchConfiguration("pose_publish_hz"),
                "vision_override_enabled": LaunchConfiguration("vision_override_enabled"),
                "vision_pick_z": LaunchConfiguration("vision_pick_z"),
                "vision_lidar_in_arm_x": LaunchConfiguration("vision_lidar_in_arm_x"),
                "vision_lidar_in_arm_y": LaunchConfiguration("vision_lidar_in_arm_y"),
                "vision_lidar_in_arm_yaw_rad": LaunchConfiguration("vision_lidar_in_arm_yaw_rad"),
            }
        ],
    )

    return LaunchDescription(
        [
            nav_state_topic_arg,
            nav_task_points_topic_arg,
            arm_mission_service_arg,
            arm_debug_state_service_arg,
            nav_arm_event_service_arg,
            nav_mission_request_topic_arg,
            nav_mission_response_topic_arg,
            nav_arm_event_request_topic_arg,
            nav_arm_event_response_topic_arg,
            pick_service_name_arg,
            target_pose_topic_arg,
            pose_auto_publish_arg,
            mock_nav_publish_enabled_arg,
            pose_publish_hz_arg,
            vision_override_enabled_arg,
            vision_pick_z_arg,
            vision_lidar_in_arm_x_arg,
            vision_lidar_in_arm_y_arg,
            vision_lidar_in_arm_yaw_rad_arg,
            nav_mock_ui,
        ]
    )
