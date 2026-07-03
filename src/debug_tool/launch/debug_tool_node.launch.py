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
            report_period_sec_arg,
            csv_enabled_arg,
            csv_dir_arg,
            motor_count_arg,
            debug_tool_node,
        ]
    )
