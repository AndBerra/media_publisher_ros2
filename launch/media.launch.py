import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Top-level launch file that includes both audio and video launch files.
    """

    # Get the package's share directory
    media_publisher_pkg_dir = get_package_share_directory("media_publisher")

    # Create the IncludeLaunchDescription action for the audio launch file
    audio_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(media_publisher_pkg_dir, "launch", "audio.launch.py")
        )
    )

    # Create the IncludeLaunchDescription action for the video launch file
    video_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(media_publisher_pkg_dir, "launch", "video.launch.py")
        )
    )

    return LaunchDescription(
        [
            audio_launch_file,
            video_launch_file,
        ]
    )
