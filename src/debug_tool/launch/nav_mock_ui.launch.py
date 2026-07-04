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
    pose_publish_hz_arg = DeclareLaunchArgument(
        "pose_publish_hz",
        default_value="10.0",
        description="Mock lidar pose publish rate",
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
                "nav_arm_event_service": LaunchConfiguration("nav_arm_event_service"),
                "nav_mission_request_topic": LaunchConfiguration("nav_mission_request_topic"),
                "nav_mission_response_topic": LaunchConfiguration("nav_mission_response_topic"),
                "nav_arm_event_request_topic": LaunchConfiguration("nav_arm_event_request_topic"),
                "nav_arm_event_response_topic": LaunchConfiguration("nav_arm_event_response_topic"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "pose_auto_publish": LaunchConfiguration("pose_auto_publish"),
                "pose_publish_hz": LaunchConfiguration("pose_publish_hz"),
            }
        ],
    )

    return LaunchDescription(
        [
            nav_state_topic_arg,
            nav_task_points_topic_arg,
            arm_mission_service_arg,
            nav_arm_event_service_arg,
            nav_mission_request_topic_arg,
            nav_mission_response_topic_arg,
            nav_arm_event_request_topic_arg,
            nav_arm_event_response_topic_arg,
            target_pose_topic_arg,
            pose_auto_publish_arg,
            pose_publish_hz_arg,
            nav_mock_ui,
        ]
    )
