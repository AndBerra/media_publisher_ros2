import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from datetime import datetime


def generate_launch_description():
    """
    Top-level launch file that includes both audio and video launch files.
    """

    # Get the package's share directory
    media_publisher_pkg_dir = get_package_share_directory("media_publisher")

    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="false",
        description="Set to 'true' to record all topics to a ROS2 bag file.",
    )

    # get rosbag as param
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="false",
        description="Set to 'true' to record all topics to a ROS2 bag file.",
    )

    # define storage folder for bags
    storage_folder = "/ros2_bags"
    # define file name
    bag_filename = "rosbag_" + datetime.now().strftime("%Y_%m_%d-%H_%M_%S")

    # get path to store file
    output_path = os.path.join(storage_folder, bag_filename)

    # This action will only be executed if the 'record_bag' launch argument is 'true'.
    bag_recorder_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_bag")),
        cmd=["ros2", "bag", "record", "-a", "-o", output_path],
        output="screen",
    )

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
            record_bag_arg,
            audio_launch_file,
            video_launch_file,
            bag_recorder_process,
        ]
    )
