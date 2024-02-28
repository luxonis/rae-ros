import logging as log
from dataclasses import dataclass
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature


@dataclass
class StateInfo:
    """
    A class for representing the robot's state.

    Attributes
    ----------
        battery_state (BatteryState): The current state of the robot's battery.
        cpu_usage (float): The current CPU usage of the robot.
        mem_usage (float): The current memory usage of the robot.
        temp (float): The current temperature of the robot.
        disk (float): The current disk usage of the robot.
        net_up (float): The current upload speed of the robot.
        net_down (float): The current download speed of the robot.

    """

    battery_state: BatteryState = BatteryState()
    cpu_usage: float = 0.0
    mem_usage: float = 0.0
    temp: float = 0.0
    disk: float = 0.0
    net_up: float = 0.0
    net_down: float = 0.0


class StateController:
    """
    A class for managing the robot's state.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        state_info (StateInfo): Stores the current state of the robot's battery.

    """

    def __init__(self, ros_interface, publish_state_info=True, display=None):
        self._ros_interface = ros_interface
        self._display = display
        self._publish_state_info = publish_state_info

        self._state_info = StateInfo()
        self._ros_interface.create_subscriber(
            "battery_status", BatteryState, self.battery_state_cb)
        self._ros_interface.create_subscriber(
            "cpu", Float32, self.cpu_usage_cb)
        self._ros_interface.create_subscriber(
            "mem", Float32, self.mem_usage_cb)
        self._ros_interface.create_subscriber(
            "temp", Temperature, self.temp_cb)
        self._ros_interface.create_subscriber(
            "disk", Float32, self.disk_cb)
        self._ros_interface.create_subscriber(
            "net_up", Float32, self.net_up_cb)
        self._ros_interface.create_subscriber(
            "net_down", Float32, self.net_down_cb)
        if self._publish_state_info:
            self._ros_interface.create_timer(
                'state_info', 1, self.state_info_cb)
        else:
            self._ros_interface.destroy_timer('state_info')

        log.info("State Controller ready")

    def battery_state_cb(self, data):
        self._state_info.battery_state = data

    def cpu_usage_cb(self, data):
        self._state_info.cpu_usage = data.data

    def mem_usage_cb(self, data):
        self._state_info.mem_usage = data.data

    def temp_cb(self, data):
        self._state_info.temp = data.temperature

    def disk_cb(self, data):
        self._state_info.disk = data.data

    def net_up_cb(self, data):
        self._state_info.net_up = data.data

    def net_down_cb(self, data):
        self._state_info.net_down = data.data

    def state_info_cb(self):
        if self._display is not None:
            self._display.add_state_overlay(self._state_info)

    @property
    def state_info(self):
        return self._state_info

    @property
    def publish_state_info(self):
        return self._publish_state_info

    @publish_state_info.setter
    def publish_state_info(self, value):
        self._publish_state_info = value
        if self._publish_state_info:
            self._ros_interface.create_timer(
                'state_info', 1, self.state_info_cb)
        else:
            self._ros_interface.destroy_timer('state_info')
