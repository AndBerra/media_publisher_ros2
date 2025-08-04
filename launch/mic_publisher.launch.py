import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the microphone publisher and the Matplotlib visualizer.
    """

    # micro publisher
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

    # audio visualizer
    audio_visualizer_node = Node(
        package="media_publisher",
        executable="audio_visualizer",
        name="audio_visualizer",
        output="screen",
    )

    return LaunchDescription([mic_publisher_node, audio_visualizer_node])
