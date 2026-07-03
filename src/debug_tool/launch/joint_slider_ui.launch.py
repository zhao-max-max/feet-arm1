from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    state_topic_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="/arm2/_lowState/joint",
        description="RobotState topic used to synchronize the joint sliders",
    )
    ready_topic_arg = DeclareLaunchArgument(
        "ready_topic",
        default_value="/robot_driver/ready",
        description="Driver readiness topic",
    )
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="move_joint",
        description="MoveJoint action name provided by control_node",
    )
    mode_service_arg = DeclareLaunchArgument(
        "mode_service",
        default_value="set_controller_mode",
        description="Controller mode service name",
    )
    default_mode_arg = DeclareLaunchArgument(
        "default_mode",
        default_value="moving",
        description="Initial controller mode selected in the UI",
    )
    max_velocity_arg = DeclareLaunchArgument(
        "max_velocity",
        default_value="0.6",
        description="Maximum joint velocity sent in MoveJoint goals",
    )
    max_acceleration_arg = DeclareLaunchArgument(
        "max_acceleration",
        default_value="1.0",
        description="Maximum joint acceleration sent in MoveJoint goals",
    )
    blend_radius_arg = DeclareLaunchArgument(
        "blend_radius",
        default_value="0.02",
        description="Blend radius sent in MoveJoint goals",
    )
    send_debounce_ms_arg = DeclareLaunchArgument(
        "send_debounce_ms",
        default_value="180",
        description="Debounce time for slider updates before sending a goal",
    )

    joint_slider_ui = Node(
        package="debug_tool",
        executable="joint_slider_ui",
        name="joint_slider_ui_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "state_topic": LaunchConfiguration("state_topic"),
                "ready_topic": LaunchConfiguration("ready_topic"),
                "action_name": LaunchConfiguration("action_name"),
                "mode_service": LaunchConfiguration("mode_service"),
                "default_mode": LaunchConfiguration("default_mode"),
                "controller_modes": ["idle", "gravity_comp", "moving", "loaded"],
                "max_velocity": LaunchConfiguration("max_velocity"),
                "max_acceleration": LaunchConfiguration("max_acceleration"),
                "blend_radius": LaunchConfiguration("blend_radius"),
                "send_debounce_ms": LaunchConfiguration("send_debounce_ms"),
                "joint_lower_deg": [-240.0, 0.0, -172.0, -115.0, -180.0],
                "joint_upper_deg": [240.0, 200.0, 10.0, 90.0, 180.0],
            }
        ],
    )

    return LaunchDescription(
        [
            state_topic_arg,
            ready_topic_arg,
            action_name_arg,
            mode_service_arg,
            default_mode_arg,
            max_velocity_arg,
            max_acceleration_arg,
            blend_radius_arg,
            send_debounce_ms_arg,
            joint_slider_ui,
        ]
    )
