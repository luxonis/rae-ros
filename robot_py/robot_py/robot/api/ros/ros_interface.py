# needed by RH to import launch
import sys
del sys.path[0]
sys.path.append('')

import logging as log
import os
import threading
import rclpy
import subprocess
import asyncio
import multiprocessing
from time import sleep
import signal
from functools import partial

from typing import Any, Callable, Dict, Type
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.timer import Timer
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

QOS_PROFILE = 10

log.basicConfig(level=log.INFO)


class ROSInterface:
    """
    A class that manages ROS2 functionalities for a robot or a system.

    It includes initializing ROS2 context, 
    creating and managing nodes, publishers, and subscribers. 
    It also handles the startup and shutdown processes 
    for ROS2.

    Attributes
    ----------
        _name (str): Name of the ROS2 node.
        _context (rclpy.context.Context | None): The ROS2 context.
        _node (rclpy.node.Node | None): The ROS2 node.
        _publishers (dict[str, Publisher]): Dictionary of ROS2 publishers.
        _subscribers (dict[str, Subscription]): Dictionary of ROS2 subscribers.
        _service_clients (dict[str, Client]): Dictionary of ROS2 service clients.
        _action_clients (dict[str, dict[ActionClient]]): Dictionary of ROS2 action clients.
        _timers (dict[str, Timer]): Dictionary of ROS2 timers.
        _tf_buffer: The TF2 buffer.
        _tf_listener: The TF2 listener.
        _executor (Executor): The ROS2 executor.
        _executor_thread (threading.Thread): The thread for the ROS2 executor.
        _launch_service (LaunchService): The ROS2 launch service.
        _stop_event (multiprocessing.Event): The event for stopping the ROS2 launch service.
        _process (multiprocessing.Process): The process for running the ROS2 launch service.

    Methods
    -------
        get_node(): Returns the current ROS2 node.
        start_hardware_process(): Starts the hardware process for ROS2.
        start(): Initializes and starts the ROS2 node and executor.
        stop_ros_process(): Stops the ROS2 hardware process.
        stop(): Shuts down the ROS2 node and context.
        create_publisher(topic_name, msg_type, qos_profile): Creates a publisher for a given topic.
        publish(topic_name, msg): Publishes a message on a given topic.
        create_subscriber(topic_name, msg_type, callback, qos_profile): Creates a subscriber for a given topic.
        create_timer(timer_name, period, callback): Creates a timer for a given topic.
        create_service_client(srv_name, srv_type): Creates a service client for a given service.
        call_async_srv(srv_name, req): Calls a service asynchronously.
        create_action_client(action_name, action_type): Creates an action client for a given action.
        call_async_action_simple(action_name, goal): Calls an action asynchronously.
        call_async_action(action_name, goal, goal_response_callback, goal_result_callback, goal_feedback_callback): Calls an action asynchronously.
    
    """

    def __init__(self, name: str, namespace='/rae') -> None:
        """
        Initialize the ROS2Manager instance.

        Args:
        ----
            name (str): The name of the ROS2 node.
            namespace (str): The namespace of the ROS2 node.

        """
        self._namespace = namespace
        self._name = name
        self._launch_service = None
        self._stop_event = None
        self._process = None
        self._context: rclpy.context.Context | None = None
        self._node: rclpy.node.Node | None = None
        self._publishers: dict[str, Publisher] = {}
        self._subscribers: dict[str, Subscription] = {}
        self._service_clients: dict[str, Client] = {}
        self._action_clients: dict[str, ActionClient] = {}
        self._timers: dict[str, Timer] = {}
        self._tf_buffer = None
        self._tf_listener = None

    def get_node(self):
        return self._node

    def start_hardware_process(self):
        """Start RAE hardware drivers in a separate process."""
        self._launch_service = LaunchService(noninteractive=True)
        ld = LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rae_hw'), 'launch', 'control.launch.py')))])
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(
            target=self._run_process, args=(self._stop_event, ld), daemon=True)
        self._process.start()

    def _run_process(self, stop_event, launch_description):
        loop = asyncio.get_event_loop()
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)

    def start(self, start_hardware) -> None:
        """
        Run RAE hardware drivers process.Initializes and starts the ROS2 node and executor. It sets up the ROS2 context and starts the ROS2 spin.
        
        Args:
        ----
            start_hardware (bool): Whether to start the hardware process or not.

        """
        if start_hardware:
            self.start_hardware_process()
        self._context = rclpy.Context()
        self._context.init()
        log.info("ROS2 context initialized.")

        self._node = rclpy.create_node(
            self._name, context=self._context, namespace=self._namespace)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        log.info(f"Created ROS2 node with name: {self._name}...")
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._spin)
        self._executor_thread.start()
        log.info(f"Node started!")

    def _spin(self):
        log.info("rlcpy thread> Start")
        try:
            self._executor.spin()
        except Exception as e:
            log.info("rlcpy thread> failed")
            log.error(e)
        log.info("rlcpy thread> Done")

    def stop_ros_process(self):
        """Stop the ROS2 hardware process by terminating the related subprocess."""
        self._stop_event.set()
        self._process.join()

    def stop(self) -> None:
        """
        Shut down RAE drivers, ROS2 node and context.

        This includes stopping the executor, destroying publishers subscribers, service clients, action clients, timers
        and shutting down the ROS2 context.
        """
        self.stop_ros_process()
        if not self._context:
            log.info("ROS2 context is already stopped")
            return

        if self._node:
            log.info("Destroying ROS2 node...")
            self._destroy_interfaces()
            self._node.destroy_node()
            self._node = None

        if self._executor_thread:
            self._executor.shutdown()
            self._executor_thread.join()
            self._executor_thread = None

        if self._context:
            log.info("Shutting down ROS2 context...")
            self._context.try_shutdown()
            self._context.destroy()
            self._context = None

    def _destroy_interfaces(self):
        for topic_name, publisher in self._publishers.items():
            log.info(f"Destroying {topic_name} publisher...")
            self._node.destroy_publisher(publisher)

        for topic_name, subscriber in self._subscribers.items():
            log.info(f"Destroying {topic_name} subscriber...")
            self._node.destroy_subscription(subscriber)

        for srv_name, client in self._service_clients.items():
            log.info(f"Destroying {srv_name} service client...")
            self._node.destroy_client(client)
        for timer_name, timer in self._timers.items():
            log.info(f"Destroying {timer_name} timer...")
            self._node.destroy_timer(timer)

        for action_name, action_client in self._action_clients.items():
            log.info(f"Destroying {action_name} action client...")
            action_client['goal_handle'].cancel_goal_async()
            self._node.destroy_client(action_client['client'])

        self._publishers.clear()
        self._subscribers.clear()
        self._service_clients.clear()
        self._timers.clear()

    def create_publisher(self, topic_name: str, msg_type: Any, qos_profile: int = QOS_PROFILE) -> None:
        if topic_name not in self._publishers:
            if msg_type is not None:
                log.info(f"Creating {topic_name} publisher")
                self._publishers[topic_name] = self._node.create_publisher(
                    msg_type, topic_name, qos_profile)
            else:
                log.warning(f"Unknown message type '{msg_type}'")
        else:
            log.warning(f"Publisher for topic '{topic_name}' already exists")

    def create_service_client(self, srv_name: str, srv_type: Any) -> None:
        if srv_name not in self._service_clients:
            if srv_type is not None:
                log.info(f"Creating {srv_name} service client")
                self._service_clients[srv_name] = self._node.create_client(
                    srv_type, srv_name)
            else:
                log.warning(f"Unknown service type '{srv_type}")

    def call_async_srv(self, srv_name: str, req: Any) -> None:
        log.info(f"Calling service {srv_name}")
        future = self._service_clients[srv_name].call_async(req)
        return future.result()

    def publish(self, topic_name: str, msg: Any) -> None:
        if topic_name in self._publishers:
            publisher = self._publishers[topic_name]
            publisher.publish(msg)
        else:
            log.warning(f"No publisher found for topic '{topic_name}'")

    def create_subscriber(self, topic_name: str, msg_type: Any, callback=None, qos_profile: int = QOS_PROFILE) -> None:
        if topic_name not in self._subscribers:
            if msg_type is not None:
                if callback is None:
                    callback = self._default_callback

                log.info(f"Creating {topic_name} subscriber")
                self._subscribers[topic_name] = self._node.create_subscription(
                    msg_type, topic_name, callback, qos_profile)
            else:
                log.warning(f"Unknown message type '{msg_type}'")

    def _default_callback(self, msg) -> None:
        log.info(f"[Default callback] Received message: {msg}")

    def create_timer(self, timer_name, period, callback) -> None:
        if timer_name not in self._timers:
            if callback is not None:
                log.info(f"Creating {timer_name} timer")
                self._timers[timer_name] = self._node.create_timer(
                    period, callback)
            else:
                log.error(f"No callback function given for timer {timer_name}")

    def create_action_client(self, action_name: str, action_type: Any) -> None:
        if action_name not in self._action_clients:
            if action_type is not None:
                log.info(f"Creating {action_name} action client")
                self._action_clients[action_name] = {}
                self._action_clients[action_name]['client'] = ActionClient(
                    self._node, action_type, action_name)
            else:
                log.warning(f"Unknown action type '{action_type}'")

    def call_async_action_simple(self, action_name: str, goal: Any) -> ClientGoalHandle:
        log.info(f"Calling action {action_name}")
        self._action_clients[action_name]['client'].wait_for_server()
        future = self._action_clients[action_name]['client'].send_goal_async(
            goal)
        while not future.done():
            log.info('Waiting for result...')
            sleep(0.5)
        return future.result()

    def call_async_action(self, action_name: str, goal: Any, goal_response_callback=None, goal_result_callback=None, goal_feedback_callback=None) -> ClientGoalHandle:
        log.info(f"Calling action {action_name}")
        if goal_response_callback is None:
            self._action_clients[action_name]['goal_response_callback'] = self._default_goal_response_callback
        else:
            self._action_clients[action_name]['goal_response_callback'] = goal_response_callback
        if goal_result_callback is None:
            self._action_clients[action_name]['goal_result_callback'] = self._default_goal_result_callback
        else:
            self._action_clients[action_name]['goal_result_callback'] = goal_result_callback

        self._action_clients[action_name]['client'].wait_for_server()

        if goal_feedback_callback is None:
            future = self._action_clients[action_name]['client'].send_goal_async(
                goal)
        else:
            future = self._action_clients[action_name]['client'].send_goal_async(
                goal, goal_feedback_callback)

        future.add_done_callback(
            partial(self._action_clients[action_name]['goal_response_callback']))

    def _default_goal_response_callback(self, future):
        action_name = '/fibonacci'
        self._action_clients[action_name]['goal_handle'] = future.result()
        if not self._action_clients[action_name]['goal_handle'].accepted:
            log.info('Goal rejected :(')
            return
        log.info('Goal accepted :)')
        get_result_future = self._action_clients[action_name]['goal_handle'].get_result_async(
        )
        get_result_future.add_done_callback(
            partial(self._action_clients[action_name]['goal_result_callback']))

    def _default_goal_result_callback(self, future):
        result = future.result().result
        log.info(f'Result received: {result}')

    def cancel_action(self, action_name: str):
        log.info(f"Cancelling action {action_name}")
        self._action_clients[action_name]['goal_handle'].cancel_goal_async()

    def get_frame_position(self, source_frame, target_frame) -> TransformStamped:
        """
        Get the position of a frame relative to another frame.

        Args:
        ----
            source_frame (str): The source frame.
            target_frame (str): The target frame.

        Returns
        -------
            TransformStamped: The position of the source frame relative to the target frame.

        """
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            return transform
        except TransformException as e:
            log.error(e)
            return None
