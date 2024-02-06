#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node

from rae_msgs.msg import LEDControl


class MockLeds(Node):
    def __init__(self):
        super().__init__('led_node')


    def led_callback(self, msg: LEDControl):
        self.get_logger().info('I heard: "%s"' % msg)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._led_sub = self.create_subscription(
            LEDControl, 'leds', self.led_callback, 10)
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
    mock_leds = MockLeds()
    rclpy.spin(mock_leds)
    rclpy.shutdown()
