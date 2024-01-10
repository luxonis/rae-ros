#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class MockLCD(Node):
    def __init__(self):
        super().__init__('mock_display')
        self._image_sub = self.create_subscription(Image, 'lcd', self.image_callback, 10)
    def image_callback(self, msg: Image):
        self.get_logger().info(f'I heard: {msg.width}, {msg.height}') 

if __name__ == '__main__':
    rclpy.init()
    mock_lcd = MockLCD()
    rclpy.spin(mock_lcd)
    mock_lcd.destroy_node()
    rclpy.shutdown()