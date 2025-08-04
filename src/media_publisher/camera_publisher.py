import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraPublisherNode(Node):
    """
    A ROS2 node that captures video from a webcam and publishes it as sensor_msgs/Image.
    """

    def __init__(self):
        super().__init__("camera_publisher_node")

        # Declare parameters
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("image_topic", "video/image_raw")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("fps", 30.0)

        # Get parameters
        camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        fps = self.get_parameter("fps").get_parameter_value().double_value

        # Create the publisher
        self.publisher_ = self.create_publisher(Image, image_topic, 10)

        # Create the OpenCV video capture object
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera with ID: {camera_id}")
            self.destroy_node()
            return

        # Create the CvBridge object
        self.bridge = CvBridge()

        # Create a timer to publish frames at the specified FPS
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Camera publisher started. Publishing to '{image_topic}' at {fps} FPS."
        )
        self.context.on_shutdown(self.on_shutdown)

    def timer_callback(self):
        """
        Called by the timer to capture and publish a single video frame.
        """
        ret, frame = self.cap.read()

        if ret:
            # Convert the OpenCV frame to a ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            # Set the header of the message
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = self.frame_id

            # Publish the message
            self.publisher_.publish(ros_image_msg)
        else:
            self.get_logger().warn("Could not retrieve frame from camera.")

    def on_shutdown(self):
        """
        Ensure the camera is released when the node is shut down.
        """
        self.get_logger().info("Shutting down camera publisher.")
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    rclpy.spin(camera_publisher_node)
    camera_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
