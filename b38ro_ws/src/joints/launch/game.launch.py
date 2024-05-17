from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Game logic code
            Node(package="joints", executable="game"),
            # Computer vision code
            Node(package="joints", executable="vision"),
        ]
    )
