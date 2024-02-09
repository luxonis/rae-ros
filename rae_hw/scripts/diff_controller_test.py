#!/usr/bin/env python3

import os
import rclpy
import time
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class DiffControllerTest(Node):
    def __init__(self):
        super().__init__('rae_hw_diff_controller_test')
        self.get_logger().info('Initializing Diff Controller Test')
        self._mock = self.declare_parameter('mock', False).value
        self._controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers')
        self.get_logger().info('Diff Controller Test initialized')

    def check_diff_controller(self):
        if self._mock:
            return
        self.get_logger().info('Checking diff controller...')
        diff_controller_running = False
        while not diff_controller_running:
            self._controllers_client.wait_for_service()
            req = ListControllers.Request()
            future = self._controllers_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                for controller in future.result().controller:
                    if controller.name == 'diff_controller' and controller.state == 'active':
                        self.get_logger().info('Diff controller running')
                        diff_controller_running = True
                        break
                else:
                    self.get_logger().error('Failed to get controller list')
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = DiffControllerTest()
    node.check_diff_controller()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
