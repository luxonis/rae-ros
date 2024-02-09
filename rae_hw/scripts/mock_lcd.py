#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node

from sensor_msgs.msg import Image

class MockLCD(Node):
    def __init__(self):
        super().__init__('lcd_node')

    def image_callback(self, msg: Image):
        self.get_logger().info(f'I heard: {msg.width}, {msg.height}') 

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._image_sub = self.create_subscription(Image, 'lcd', self.image_callback, 10)
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
    mock_lcd = MockLCD()
    rclpy.spin(mock_lcd)
    rclpy.shutdown()