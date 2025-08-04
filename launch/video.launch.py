from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches video-related nodes: camera publisher and RQT Image Viewer.
    """

    # Camera Publisher
    camera_publisher_node = Node(
        package="media_publisher",
        executable="camera_publisher",
        name="camera_publisher_node",
        output="screen",
        parameters=[
            {
                "camera_id": 0,
            }
        ],
    )

    # RQT Image Viewer
    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view_node",
        remappings=[("/image", "/video/image_raw")],
    )

    return LaunchDescription(
        [
            camera_publisher_node,
            rqt_image_view_node,
        ]
    )
