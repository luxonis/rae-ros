#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rae_msgs.msg import LEDControl

class MockLeds(Node):
    def __init__(self):
        super().__init__('mock_leds')
        self._led_sub = self.create_subscription(LEDControl, 'leds', self.led_callback, 10)
    def led_callback(self, msg: LEDControl):
        self.get_logger().info('I heard: "%s"' % msg.data)

if __name__ == '__main__':
    rclpy.init()
    mock_leds = MockLeds()
    rclpy.spin(mock_leds)
    mock_leds.destroy_node()
    rclpy.shutdown()