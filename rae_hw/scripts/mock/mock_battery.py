#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node

from sensor_msgs.msg import BatteryState

class MockBattery(Node):
    def __init__(self):
        super().__init__('battery_node')

    def timer_callback(self):
        msg = BatteryState()
        self._battery_pub.publish(msg)
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
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
    mock_battery = MockBattery()
    rclpy.spin(mock_battery)
    rclpy.shutdown()