#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio


class MockMic(Node):
    def __init__(self):
        super().__init__('mock_mic')
        self._audio_pub = self.create_publisher(Audio, 'audio_in', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = Audio()
        self._audio_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    mock_mic = MockMic()
    rclpy.spin(mock_mic)
    mock_mic.destroy_node()
    rclpy.shutdown()