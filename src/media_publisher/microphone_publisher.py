import rclpy
from rclpy.node import Node
import pyaudio
from std_msgs.msg import UInt8MultiArray


class MicrophonePublisher(Node):
    """
    A robust ROS2 node that captures audio from a microphone and publishes it.
    It intelligently selects a microphone but allows the user to override the
    choice with a ROS parameter.
    """

    def __init__(self):
        super().__init__("microphone_publisher")

        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("channels", 1)
        self.declare_parameter("rate", 44100)
        self.declare_parameter("audio_topic", "audio")
        self.declare_parameter("device", "auto")

        self.chunk_size = (
            self.get_parameter("chunk_size").get_parameter_value().integer_value
        )
        self.channels = (
            self.get_parameter("channels").get_parameter_value().integer_value
        )
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )
        device_param = self.get_parameter("device").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(UInt8MultiArray, audio_topic, 10)
        self.p = None
        self.stream = None

        try:
            self.p = pyaudio.PyAudio()
            input_device_index = self._select_input_device(device_param)
            if input_device_index is None:
                raise RuntimeError("No suitable microphone found.")

            if not self._is_format_supported(input_device_index):
                raise RuntimeError(
                    "Microphone does not support the requested audio format."
                )

            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk_size,
                input_device_index=input_device_index,
            )
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")
            self.destroy_node()  # Mark node for cleanup
            return

        timer_period = float(self.chunk_size) / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Microphone publisher started. Publishing to '{audio_topic}'."
        )
        self.context.on_shutdown(self.on_shutdown)

    def _select_input_device(self, device_param):
        self.get_logger().info("--- Available Audio Input Devices ---")
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info.get("maxInputChannels") > 0:
                self.get_logger().info(f"  Index {i}: {info.get('name')}")
        self.get_logger().info("------------------------------------")

        if device_param != "auto":
            try:
                device_index = int(device_param)
                self.get_logger().info(f"User selected device index: {device_index}")
                return device_index
            except ValueError:
                self.get_logger().error(
                    f"Invalid device parameter '{device_param}'. It must be 'auto' or an integer."
                )
                return None

        self.get_logger().info("Searching for a suitable microphone (mode: auto)...")
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info.get("maxInputChannels") > 0:
                name = info.get("name", "").lower()
                if "pulse" in name or "default" in name or "jack" in name:
                    continue
                self.get_logger().info(
                    f"Auto-selected microphone: '{info['name']}' (Index {i})"
                )
                return i

        self.get_logger().error("Auto-selection failed: No physical microphone found.")
        return None

    def _is_format_supported(self, device_index):
        info = self.p.get_device_info_by_index(device_index)
        try:
            is_supported = self.p.is_format_supported(
                self.rate,
                input_device=device_index,
                input_channels=self.channels,
                input_format=pyaudio.paInt16,
            )
            if is_supported:
                self.get_logger().info(
                    f"Audio settings ({self.rate}Hz, {self.channels}ch) supported by device."
                )
            return is_supported
        except ValueError:
            return False

    def timer_callback(self):
        try:
            data_bytes = self.stream.read(self.chunk_size, exception_on_overflow=False)
            msg = UInt8MultiArray(data=data_bytes)
            self.publisher_.publish(msg)
        except IOError as e:
            self.get_logger().error(f"Error reading from stream: {e}")

    def on_shutdown(self):
        self.get_logger().info("Shutting down.")
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
        if self.p is not None:
            self.p.terminate()


def main(args=None):
    """The main entry point for the node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MicrophonePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"An unhandled exception occurred: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
