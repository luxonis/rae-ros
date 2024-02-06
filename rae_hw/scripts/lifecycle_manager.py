#!/usr/bin/env python3

import os
import rclpy
import cv2

from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rae_msgs.msg import LEDControl, ColorPeriod
from sensor_msgs.msg import Image
from rae_msgs.srv import PlayAudio
from std_msgs.msg import String
from controller_manager_msgs.srv import ListControllers
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory


class LifecycleManager(Node):
    def __init__(self):
        super().__init__('rae_hw_lifecycle_manager')
        self.get_logger().info('Initializing Lifecycle Manager')
        self._node_names = ['mic_node', 'battery_node', 'speakers_node']

        # for each node, create a service client to change state
        self._change_state_clients = {}
        for node_name in self._node_names:
            self._change_state_clients[node_name] = self.create_client(
                ChangeState, node_name + '/change_state')
        self._change_state_clients['led_node'] = self.create_client(
            ChangeState, 'led_node/change_state')
        self._change_state_clients['lcd_node'] = self.create_client(
            ChangeState, 'lcd_node/change_state')
        self._controllers_client = self.create_client(
            ListControllers, 'controller_manager/list_controllers')
        self._led_pub = self.create_publisher(LEDControl, 'leds', 10)
        self._lcd_pub = self.create_publisher(Image, 'lcd', 10)
        self._mock = self.declare_parameter('mock', False).value
        self._progress = 0
        self._bridge = CvBridge()
        self._ready_publisher = self.create_publisher(String, 'ready', 10)
        self._audio_client = self.create_client(PlayAudio, 'play_audio')
        self.get_logger().info('Lifecycle Manager initialized')

    def update_progress_indicators(self):
        self.get_logger().info('Updating indicators...')
        color = ColorPeriod()
        color.color.a = 1.0
        color.color.r = 1.0
        color.frequency = 1.0

        msg = LEDControl()
        msg.data = [color]
        msg.control_type = LEDControl.CTRL_TYPE_SPINNER
        msg.animation_size = 1
        msg.animation_quantity = self._progress
        self._led_pub.publish(msg)

        img = cv2.imread(os.path.join(get_package_share_directory(
            'rae_hw'), 'assets', 'rae-logo-white.jpg'))

        # add text to top of the image (image size is 160x80)
        cv2.putText(img, 'LOADING', (45, 16),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, (150, 240, 110), 1, cv2.LINE_AA)

        # add loading bar to the bottom of the image
        # add loading bar border
        cv2.rectangle(img, (18, 60), (142, 76), (255, 255, 255), 1)

        cv2.rectangle(
            img, (20, 62), (20 + self._progress*40, 74), (150, 240, 110), -1)

        img_msg = self._bridge.cv2_to_imgmsg(img, 'bgr8')
        self._lcd_pub.publish(img_msg)
        self._progress += 1

    def startup(self):
        self.get_logger().info('Starting up...')
        self.startup_visual_nodes()
        self.startup_rest()
        if not self._mock:
            self.check_diff_controller()

        img = cv2.imread(os.path.join(get_package_share_directory(
            'rae_hw'), 'assets', 'rae-logo-white.jpg'))

        # add text to top of the image (image size is 160x80)
        cv2.putText(img, 'READY', (54, 16),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, (150, 240, 110), 1, cv2.LINE_AA)

        img_msg = self._bridge.cv2_to_imgmsg(img, 'bgr8')

        self._lcd_pub.publish(img_msg)
        
        color = ColorPeriod()
        color.color.a = 1.0
        color.color.g = 1.0
        color.frequency = 1.0

        msg = LEDControl()
        msg.data = [color]
        msg.control_type = LEDControl.CTRL_TYPE_ALL
        msg.animation_size = 1
        msg.animation_quantity = 40
        self._led_pub.publish(msg)

        # send startup sound
        req = PlayAudio.Request()
        req.mp3_file = os.path.join(get_package_share_directory(
            'rae_hw'), 'assets', 'startup.mp3')
        self._audio_client.wait_for_service()
        future = self._audio_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)



        self._ready_publisher.publish(String(data='ready'))
        self.get_logger().info('Startup complete')

    def send_transition(self, node_name, transition):
        self.get_logger().info(
            f'Sending transition {transition} to {node_name}...')
        self._change_state_clients[node_name].wait_for_service()
        transition_msg = Transition()
        transition_msg.id = transition

        req = ChangeState.Request()
        req.transition = transition_msg
        future = self._change_state_clients[node_name].call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f'Successfully transitioned {node_name} to {transition}')
        else:
            self.get_logger().error(
                f'Failed to transition {node_name} to {transition}')

    def configure_and_activate(self, node_name):
        self.send_transition(node_name, Transition.TRANSITION_CONFIGURE)
        self.send_transition(node_name, Transition.TRANSITION_ACTIVATE)

    def startup_visual_nodes(self):
        self.get_logger().info('Startup LED node')
        self.configure_and_activate('led_node')
        self.configure_and_activate('lcd_node')
        self.update_progress_indicators()

    def startup_rest(self):
        i = 1
        for node_name in self._node_names:
            self.get_logger().info(f'Startup {node_name}')
            self.send_transition(node_name, Transition.TRANSITION_CONFIGURE)
            self.send_transition(
                node_name, Transition.TRANSITION_ACTIVATE)
            self.update_progress_indicators()

    def check_diff_controller(self):
        self.get_logger().info('Checking diff controller...')
        diff_controller_running = False
        while not diff_controller_running:
            self._controllers_client.wait_for_service()
            req = ListControllers.Request()
            future = self._controllers_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                for controller in future.result().controller:
                    if controller.name == 'diff_controller':
                        if controller.state == 'active':
                            self.get_logger().info('Diff controller running')
                        diff_controller_running = True
                        break
            else:
                self.get_logger().error('Failed to get controller list')



def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManager()
    node.startup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
