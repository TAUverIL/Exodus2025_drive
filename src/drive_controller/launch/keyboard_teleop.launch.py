"""Standalone keyboard teleop. Run alongside ackermann.launch.py."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="drive_controller",
                executable="keyboard_teleop_node.py",
                name="keyboard_teleop",
                output="screen",
                # Keep the terminal attached so key presses reach the node.
                prefix="xterm -e",
                emulate_tty=True,
            ),
        ]
    )
