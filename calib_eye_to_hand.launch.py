""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-TO-HAND: base_link -> camera_color_optical_frame """
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
                "base_link",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.249838",
                "--y",
                "0.252288",
                "--z",
                "0.741697",
                "--qx",
                "-0.00149394",
                "--qy",
                "0.980578",
                "--qz",
                "-0.196103",
                "--qw",
                "-0.00285558",
                # "--roll",
                # "2.74682",
                # "--pitch",
                # "-0.00501432",
                # "--yaw",
                # "3.13754",
            ],
        ),
    ]
    return LaunchDescription(nodes)
