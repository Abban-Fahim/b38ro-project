from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch the rosbridge websocket server for web gui
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("rosbridge_server"),
                            "launch",
                            "rosbridge_websocket_launch.xml",
                        ]
                    )
                )
            ),
            Node(package="joints", executable="talker"),
        ]
    )
