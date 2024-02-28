#!/usr/bin/env python3

import psutil
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import TransitionCallbackReturn, Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature

class SysInfoNode(Node):
    def __init__(self):
        super().__init__('sys_info_node')
        self._prev_bytes_sent = 0
        self._prev_bytes_recv = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')
        self._cpu_pub = self.create_publisher(Float32, 'cpu', 10)
        self._mem_pub = self.create_publisher(Float32, 'mem', 10)
        self._temp_pub = self.create_publisher(Temperature, 'temp', 10)
        self._net_up_pub = self.create_publisher(Float32, 'net_up', 10)
        self._net_down_pub = self.create_publisher(Float32, 'net_down', 10)
        self._disk_pub = self.create_publisher(Float32, 'disk', 10)
        self._mock = self.declare_parameter('mock', False).value
        
        self._timer = self.create_timer(1, self.timer_callback)
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

    def timer_callback(self):
        cpu = psutil.cpu_percent()
        if cpu > 90:
            self.get_logger().warn(f'CPU usage is {cpu}%')
        mem = psutil.virtual_memory().percent
        if mem > 90:
            self.get_logger().warn(f'Memory usage is {mem}%')
        temp = psutil.sensors_temperatures()
        net = psutil.net_io_counters()
        disk = psutil.disk_usage('/').percent
        self._cpu_pub.publish(Float32(data=cpu))
        self._mem_pub.publish(Float32(data=mem))
        self._disk_pub.publish(Float32(data=disk))
        if not self._mock:
            curr_temp = temp['bq27441_0'][0].current
            if curr_temp > 50:
                self.get_logger().warn(f'Temperature is {curr_temp}°C')
            self._temp_pub.publish(Temperature(temperature=curr_temp))

        mbs_up = (net.bytes_sent - self._prev_bytes_sent) / 1024 / 1024
        mbs_down = (net.bytes_recv - self._prev_bytes_recv) / 1024 / 1024

        self._net_up_pub.publish(Float32(data=mbs_up))
        self._net_down_pub.publish(Float32(data=mbs_down))

        self._prev_bytes_sent = net.bytes_sent
        self._prev_bytes_recv = net.bytes_recv

if __name__ == '__main__':
    rclpy.init()
    sys_info_node = SysInfoNode()
    rclpy.spin(sys_info_node)
    rclpy.shutdown()
