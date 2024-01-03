#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState

class MockBattery(Node):
    def __init__(self):
        super().__init__('mock_battery')
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = BatteryState()
        self._battery_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    mock_battery = MockBattery()
    rclpy.spin(mock_battery)
    mock_battery.destroy_node()
    rclpy.shutdown()