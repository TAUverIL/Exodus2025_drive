"""Bring up the joy driver + Xbox teleop node together."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"deadzone": 0.05, "autorepeat_rate": 20.0}],
        output="screen",
    )

    xbox_teleop_node = Node(
        package="drive_controller",
        executable="xbox_teleop_node.py",
        name="xbox_teleop",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([joy_node, xbox_teleop_node])
