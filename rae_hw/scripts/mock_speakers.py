#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from audio_msgs.msg import Audio
from rae_msgs.srv import PlayAudio


class MockSpeakers(Node):
    def __init__(self):
        super().__init__('mock_speakers')
        self._audio_sub = self.create_subscription(
            Audio, 'audio_out', self.audio_callback, 10)
        self._play_service = self.create_service(
            PlayAudio, 'play_audio', self.play_callback)

    def audio_callback(self, msg: Audio):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def play_callback(self, request: PlayAudio.Request, response):
        self.get_logger().info('I heard: "%s"' % request.mp3_file)
        return response


if __name__ == '__main__':
    rclpy.init()
    mock_speakers = MockSpeakers()
    rclpy.spin(mock_speakers)
    mock_speakers.destroy_node()
    rclpy.shutdown()
