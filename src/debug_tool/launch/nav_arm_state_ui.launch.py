from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arm_debug_state_service_arg = DeclareLaunchArgument(
        "arm_debug_state_service",
        default_value="/arm/debug_state_command",
        description="Debug state service provided by arm task_node",
    )
    initial_task_index_arg = DeclareLaunchArgument(
        "initial_task_index",
        default_value="1",
        description="Initial task_index shown in the UI",
    )

    nav_arm_state_ui = Node(
        package="debug_tool",
        executable="nav_arm_state_ui",
        name="nav_arm_state_ui_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "arm_debug_state_service": LaunchConfiguration("arm_debug_state_service"),
                "initial_task_index": LaunchConfiguration("initial_task_index"),
            }
        ],
    )

    return LaunchDescription(
        [
            arm_debug_state_service_arg,
            initial_task_index_arg,
            nav_arm_state_ui,
        ]
    )
