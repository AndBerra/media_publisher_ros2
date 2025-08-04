import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class AudioVisualizerNode(Node):
    """
    Subscribes to raw audio data and visualizes it in real-time
    as a waveform using Matplotlib.
    """

    def __init__(self):
        super().__init__("audio_visualizer")

        self.declare_parameter("chunk_size", 1024)
        self.declare_parameter("audio_topic", "audio")

        self.chunk_size = (
            self.get_parameter("chunk_size").get_parameter_value().integer_value
        )
        audio_topic = (
            self.get_parameter("audio_topic").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            UInt8MultiArray, audio_topic, self.audio_callback, 10
        )

        self.fig, self.ax = plt.subplots()
        self.x_axis = np.arange(self.chunk_size)

        self.ax.set_ylim(-32768, 32767)
        self.ax.set_xlim(0, self.chunk_size)
        self.ax.set_title("Real-time Audio Waveform")
        (self.line,) = self.ax.plot(self.x_axis, np.zeros(self.chunk_size))

        self.audio_data_buffer = np.zeros(self.chunk_size, dtype=np.int16)
        self.get_logger().info("Audio visualizer started. Close plot to stop.")

    def audio_callback(self, msg: UInt8MultiArray):
        self.audio_data_buffer = np.frombuffer(msg.data, dtype=np.int16)

    def update_plot(self, frame):
        rclpy.spin_once(self, timeout_sec=0)
        self.line.set_ydata(self.audio_data_buffer)
        return (self.line,)


def main(args=None):
    rclpy.init(args=args)
    node = AudioVisualizerNode()
    ani = animation.FuncAnimation(node.fig, node.update_plot, blit=True, interval=10)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
