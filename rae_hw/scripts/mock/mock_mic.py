#!/usr/bin/env python3
from time import sleep
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node
from audio_msgs.msg import Audio


class MockMic(Node):
    def __init__(self):
        super().__init__('mic_node')

    def timer_callback(self):
        msg = Audio()
        self._audio_pub.publish(msg)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._audio_pub = self.create_publisher(Audio, 'audio_in', 10)
        self.timer = self.create_timer(1, self.timer_callback)
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
    mock_mic = MockMic()
    rclpy.spin(mock_mic)
    rclpy.shutdown()