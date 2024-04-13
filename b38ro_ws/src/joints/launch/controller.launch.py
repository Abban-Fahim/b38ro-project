from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
import os


def shutdown_cb(event, context):
    os.system("echo 'implement switching arm controller for real arm'")
    return [LogInfo(msg="Shutdown due to " + event.reason)]


def generate_launch_description():
    return LaunchDescription(
        [
            # Node that reads controller states
            Node(package="joy", executable="joy_node", name="joy_node"),
            # Node that sends commands to robot
            Node(package="joints", executable="controller"),
            RegisterEventHandler(OnShutdown(on_shutdown=shutdown_cb)),
        ]
    )
