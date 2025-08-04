from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches audio-related nodes: microphone publisher and audio visualizer.
    """

    # Microphone Publisher
    mic_publisher_node = Node(
        package="media_publisher",
        executable="mic_publisher",
        name="mic_publisher",
        output="screen",
        parameters=[
            {
                "chunk_size": 1024,
                "rate": 44100,
            }
        ],
    )

    # Audio Visualizer
    audio_visualizer_node = Node(
        package="media_publisher",
        executable="audio_visualizer",
        name="audio_visualizer",
        output="screen",
    )

    return LaunchDescription(
        [
            mic_publisher_node,
            audio_visualizer_node,
        ]
    )
