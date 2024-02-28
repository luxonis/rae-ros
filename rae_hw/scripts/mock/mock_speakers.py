#!/usr/bin/env python3
from time import sleep
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node

from audio_msgs.msg import Audio
from rae_msgs.srv import PlayAudio

class MockSpeakers(Node):
    def __init__(self):
        super().__init__('speakers_node')

    def audio_callback(self, msg: Audio):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def play_callback(self, request: PlayAudio.Request, response):
        self.get_logger().info('I heard: "%s"' % request.file_location)
        return response
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._audio_sub = self.create_subscription(
            Audio, 'audio_out', self.audio_callback, 10)
        self._play_service = self.create_service(
            PlayAudio, 'play_audio', self.play_callback)
        sleep(0.5)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down')
        return TransitionCallbackReturn.SUCCESS

if __name__ == '__main__':
    rclpy.init()
    mock_speakers = MockSpeakers()
    rclpy.spin(mock_speakers)
    rclpy.shutdown()
