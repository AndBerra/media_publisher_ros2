import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generates the launch description for the microphone publisher node.

    This launch file starts the mic_publisher node and allows for easy
    configuration of its parameters.
    """

    # Define the node
    mic_publisher_node = Node(
        package="media_publisher",
        executable="mic_publisher",
        name="mic_publisher",
        output="screen",
        parameters=[
            {
                "chunk_size": 1024,
                "rate": 44100,
                "channels": 1,
                "audio_topic": "audio",
            }
        ],
    )

    return LaunchDescription([mic_publisher_node])
