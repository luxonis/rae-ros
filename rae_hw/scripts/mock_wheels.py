#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped

class MockWheels(Node):
    def __init__(self):
        super().__init__('mock_wheels')
        self._odom_pub = self.create_publisher(Odometry, 'diff_controller/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(100, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(f'I heard: {msg.angular.x}, {msg.angular.z}')

    def timer_callback(self):
        msg = Odometry()
        tf = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        self._tf_broadcaster.sendTransform(tf)
        self._odom_pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    mock_wheels = MockWheels()
    rclpy.spin(mock_wheels)
    mock_wheels.destroy_node()
    rclpy.shutdown()