""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: tool0 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "tool0",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "0.0499214",
                "--y",
                "0.0509616",
                "--z",
                "0.0120582",
                "--qx",
                "-0.00518834",
                "--qy",
                "-0.0044894",
                "--qz",
                "0.99997",
                "--qw",
                "0.00361787",
                # "--roll",
                # "0.00894159",
                # "--pitch",
                # "-0.010409",
                # "--yaw",
                # "3.1344",
            ],
        ),
    ]
    return LaunchDescription(nodes)
