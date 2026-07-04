from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    state_topic_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="/arm2/_lowState/joint",
        description="RobotState feedback topic to record",
    )
    command_topic_arg = DeclareLaunchArgument(
        "command_topic",
        default_value="/arm2/_lowCmd/command",
        description="RobotCommand control topic to record",
    )
    ready_topic_arg = DeclareLaunchArgument(
        "ready_topic",
        default_value="/robot_driver/ready",
        description="Driver readiness topic to record",
    )
    nav_state_topic_arg = DeclareLaunchArgument(
        "nav_state_topic",
        default_value="/navigation/state",
        description="Navigation odometry topic used by nav_task_interface",
    )
    nav_task_points_topic_arg = DeclareLaunchArgument(
        "nav_task_points_topic",
        default_value="/navigation/task_points",
        description="Navigation task-point topic used by nav_task_interface",
    )
    arm_mission_service_arg = DeclareLaunchArgument(
        "arm_mission_service",
        default_value="/arm/mission_event",
        description="MissionCommand service provided by nav_task_interface",
    )
    nav_arm_event_service_arg = DeclareLaunchArgument(
        "nav_arm_event_service",
        default_value="/navigation/arm_event",
        description="StringCommand service consumed by nav_task_interface",
    )
    nav_mission_request_topic_arg = DeclareLaunchArgument(
        "nav_mission_request_topic",
        default_value="/debug/nav/mission_request",
        description="Debug topic mirroring MissionCommand requests",
    )
    nav_mission_response_topic_arg = DeclareLaunchArgument(
        "nav_mission_response_topic",
        default_value="/debug/nav/mission_response",
        description="Debug topic mirroring MissionCommand responses",
    )
    nav_arm_event_request_topic_arg = DeclareLaunchArgument(
        "nav_arm_event_request_topic",
        default_value="/debug/nav/arm_event_request",
        description="Debug topic mirroring arm_event requests",
    )
    nav_arm_event_response_topic_arg = DeclareLaunchArgument(
        "nav_arm_event_response_topic",
        default_value="/debug/nav/arm_event_response",
        description="Debug topic mirroring arm_event responses",
    )
    timing_event_topic_arg = DeclareLaunchArgument(
        "timing_event_topic",
        default_value="/debug/arm_task_timing",
        description="Timing event topic published by arm2_task",
    )
    report_period_sec_arg = DeclareLaunchArgument(
        "report_period_sec",
        default_value="1.0",
        description="Terminal report and CSV sample period in seconds",
    )
    csv_enabled_arg = DeclareLaunchArgument(
        "csv_enabled",
        default_value="true",
        description="Enable CSV export",
    )
    csv_dir_arg = DeclareLaunchArgument(
        "csv_dir",
        default_value="debug_tool_logs",
        description="Directory for timestamped CSV log files",
    )
    motor_count_arg = DeclareLaunchArgument(
        "motor_count",
        default_value="5",
        description="Number of arm motors to record",
    )

    debug_tool_node = Node(
        package="debug_tool",
        executable="debug_tool_node",
        name="debug_tool_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "state_topic": LaunchConfiguration("state_topic"),
                "command_topic": LaunchConfiguration("command_topic"),
                "ready_topic": LaunchConfiguration("ready_topic"),
                "nav_state_topic": LaunchConfiguration("nav_state_topic"),
                "nav_task_points_topic": LaunchConfiguration("nav_task_points_topic"),
                "arm_mission_service": LaunchConfiguration("arm_mission_service"),
                "nav_arm_event_service": LaunchConfiguration("nav_arm_event_service"),
                "nav_mission_request_topic": LaunchConfiguration("nav_mission_request_topic"),
                "nav_mission_response_topic": LaunchConfiguration("nav_mission_response_topic"),
                "nav_arm_event_request_topic": LaunchConfiguration("nav_arm_event_request_topic"),
                "nav_arm_event_response_topic": LaunchConfiguration("nav_arm_event_response_topic"),
                "timing_event_topic": LaunchConfiguration("timing_event_topic"),
                "report_period_sec": LaunchConfiguration("report_period_sec"),
                "csv_enabled": LaunchConfiguration("csv_enabled"),
                "csv_dir": LaunchConfiguration("csv_dir"),
                "motor_count": LaunchConfiguration("motor_count"),
            }
        ],
    )

    return LaunchDescription(
        [
            state_topic_arg,
            command_topic_arg,
            ready_topic_arg,
            nav_state_topic_arg,
            nav_task_points_topic_arg,
            arm_mission_service_arg,
            nav_arm_event_service_arg,
            nav_mission_request_topic_arg,
            nav_mission_response_topic_arg,
            nav_arm_event_request_topic_arg,
            nav_arm_event_response_topic_arg,
            timing_event_topic_arg,
            report_period_sec_arg,
            csv_enabled_arg,
            csv_dir_arg,
            motor_count_arg,
            debug_tool_node,
        ]
    )
