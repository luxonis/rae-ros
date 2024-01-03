#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MockWheels(Node):
    def __init__(self):
        super().__init__('mock_wheels')
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f'I heard: {msg.angular.x}, {msg.angular.z}')

    def timer_callback(self):
        msg = Odometry()
        self._odom_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    mock_wheels = MockWheels()
    rclpy.spin(mock_wheels)
    mock_wheels.destroy_node()
    rclpy.shutdown()