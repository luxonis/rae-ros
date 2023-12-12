import logging as log
import os
import threading
import rclpy
import time
import math
import subprocess
from time import sleep
import signal

from typing import Any, Callable, Dict, Type
from rclpy.executors import Executor, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.timer import Timer
from ament_index_python.packages import get_package_share_directory

log.basicConfig(level = log.INFO)
QOS_PROFILE = 10
ROS2_NAMESPACE = '/rae'


class ROS2Manager:
    """
    A class that manages ROS2 functionalities for a robot or a system. It includes initializing ROS2 context, 
    creating and managing nodes, publishers, and subscribers. It also handles the startup and shutdown processes 
    for ROS2.

    Attributes:
        ros_proc: Process for running ROS2 hardware-related commands.
        _name (str): Name of the ROS2 node.
        _context (rclpy.context.Context | None): The ROS2 context.
        _node (rclpy.node.Node | None): The ROS2 node.
        _publishers (dict[str, Publisher]): Dictionary of ROS2 publishers.
        _subscribers (dict[str, Subscription]): Dictionary of ROS2 subscribers.

    Methods:
        get_node(): Returns the current ROS2 node.
        start_hardware_process(): Starts the hardware process for ROS2.
        start(): Initializes and starts the ROS2 node and executor.
        stop_ros_process(): Stops the ROS2 hardware process.
        stop(): Shuts down the ROS2 node and context.
        create_publisher(topic_name, msg_type, qos_profile): Creates a publisher for a given topic.
        publish(topic_name, msg): Publishes a message on a given topic.
        create_subscriber(topic_name, msg_type, callback, qos_profile): Creates a subscriber for a given topic.
    """
    def __init__(self, name: str) -> None:
        """
        Initializes the ROS2Manager instance.

        Args:
            name (str): The name of the ROS2 node.
        """

        self.ros_proc = None
        self._name = name
        self._context: rclpy.context.Context | None = None
        self._node: rclpy.node.Node | None = None
        self._publishers: dict[str, Publisher] = {}
        self._subscribers: dict[str, Subscription] = {}
        self._service_clients: dict[str, Client] = {}
        self._timers: dict[str, Timer] = {}

    def get_node(self):
        return self._node

    def start_hardware_process(self):
        """
        Starts RAE hardware drivers in a separate process.
        """
        env = dict(os.environ)
        script_name = os.path.join(get_package_share_directory('robot_py'), 'scripts', 'start_ros.sh')
        self.ros_proc = subprocess.Popen(
            f"bash -c 'chmod +x {script_name} ; {script_name}'", shell=True, env=env, preexec_fn=os.setsid
        )

    def start(self, start_hardware) -> None:
        """
        Runs RAE hardware drivers process.Initializes and starts the ROS2 node and executor. It sets up the ROS2 context and starts the ROS2 spin.
        """
        if start_hardware:
            self.start_hardware_process()
            sleep(2)
        self._context = rclpy.Context()
        self._context.init()
        log.info("ROS2 context initialized.")

        self._node = rclpy.create_node(self._name, context=self._context, namespace=ROS2_NAMESPACE)
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
        """
        Stops the ROS2 hardware process by terminating the related subprocess.
        """

        if self.ros_proc is not None:
            pgid = os.getpgid(self.ros_proc.pid)
            os.killpg(pgid, signal.SIGTERM)

    def stop(self) -> None:
        """
        Shuts down RAE drivers, ROS2 node and context. This includes stopping the executor, destroying publishers and subscribers,
        and shutting down the ROS2 context.
        """
        self.stop_ros_process()
        if not self._context:
            log.info("ROS2 context is already stopped")
            return

        if self._executor_thread:
            self._executor.shutdown()
            self._executor_thread.join()
            self._executor_thread = None

        if self._node:
            log.info("Destroying ROS2 node...")
            self._destroy_interfaces()
            self._node.destroy_node()
            self._node = None

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

        self._publishers.clear()
        self._subscribers.clear()
        self._service_clients.clear()
        self._timers.clear()

    def create_publisher(self, topic_name: str, msg_type:Any, qos_profile: int = QOS_PROFILE) -> None:
        if topic_name not in self._publishers:
            if msg_type is not None:
                log.info(f"Creating {topic_name} publisher")
                self._publishers[topic_name] = self._node.create_publisher(msg_type, topic_name, qos_profile)
            else:
                log.warning(f"Unknown message type '{msg_type}'")
        else:
            log.warning(f"Publisher for topic '{topic_name}' already exists")
    
    def create_service_client(self, srv_name: str, srv_type: Any) -> None:
        if srv_name not in self._service_clients:
            if srv_type is not None:
                log.info(f"Creating {srv_name} service client")
                self._service_clients[srv_name] = self._node.create_client(srv_type, srv_name)
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
                self._subscribers[topic_name] = self._node.create_subscription(msg_type, topic_name, callback, qos_profile)
            else:
                log.warning(f"Unknown message type '{msg_type_str}'")

    def _default_callback(self, msg) -> None:
        log.info(f"[Default callback] Received message: {msg}")
        
    def create_timer(self, timer_name, period, callback) -> None:
        if timer_name not in self._timers:
            if callback is not None:
                log.info(f"Creating {timer_name} timer")
                self._timers[timer_name] = self._node.create_timer(period, callback)
            else:
                log.error(f"No callback function given for timer {timer_name}")    
