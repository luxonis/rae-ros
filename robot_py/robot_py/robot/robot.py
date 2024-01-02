from typing import Optional, List, Tuple
import logging as log
from .api.ros.ros_interface import ROSInterface
from .display import DisplayController
from .led import LEDController
from .movement import MovementController
from .audio import AudioController
from sensor_msgs.msg import BatteryState
from .perception.perception_system import PerceptionSystem

log.basicConfig(level=log.INFO)
class Robot:
    """
    A class representing a robot, integrating various controllers for movement, display, and LED management,
    and interfacing with ROS2 for communication and control.

    Attributes:
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        battery_state (BatteryState): Stores the current state of the robot's battery.
        led_controller (LEDController): Controls the robot's LEDs.
        display_controller (DisplayController): Manages the robot's display.
        movement_controller (MovementController): Handles the robot's movement.
        audio_controller (AudioController): Controls the robot's audio.
        perception_system (PerceptionSystem): Handles the robot's perception system.

    Methods:
        battery_state_cb(data): Callback method for updating battery state.
        start(): Initializes the robot's components and starts ROS2 communications.
        stop(): Stops the ROS2 communications and shuts down the robot's components.
    """

    def __init__(self, start_hardware=True, name='rae_api', namespace='/rae'):
        """
        Initializes the Robot instance.

        Args:
            start_hardware (bool, optional): If True, starts the robot's hardware components. Defaults to True.
            name (str, optional): The name of the ROS2 node. Defaults to 'rae_api'.
            namespace (str, optional): The namespace of the ROS2 node. Defaults to '/rae'.
        """
        self._name = name
        self._namespace = namespace
        self._ros_interface = ROSInterface(self._name, self._namespace)
        self._ros_interface.start(start_hardware)
        self._led_controller = LEDController(self._ros_interface)
        self._display_controller = DisplayController(self._ros_interface)
        self._movement_controller = MovementController(self._ros_interface)
        self._audio_controller = AudioController(self._ros_interface)
        self._ros_interface.create_subscriber(
            "/battery_status", BatteryState, self.battery_state_cb)
        # self._perception_system = PerceptionSystem()

        log.info('Robot ready')
    def __del__(self) -> None:
        self.stop()

    def stop(self):
        """
        Stops the ROS2 communications and deactivates the robot's controllers.
        Ensures a clean shutdown of all components.
        """
        # self._perception_system.stop()
        if self._display_controller is not None:
            self._display_controller.stop()
            self._ros_interface.stop()

    def battery_state_cb(self, data):
        self._battery_state = data
    @property
    def battery_state(self):
        """
        Retrieves the current state of the robot's battery.

        Returns:
            BatteryState: The current state of the battery.
        """
        return self._battery_state
    @property
    def perception_system(self) -> PerceptionSystem:
        """
        Returns the robot's perception system.
        """
        return self._perception_system
    @property
    def ros_interface(self) -> ROSInterface:
        """
        Returns the robot's ROS2 interface manager.
        """
        return self._ros_interface
    @property
    def led_controller(self) -> LEDController:
        """
        Returns the robot's LED controller.
        """
        return self._led_controller
    @property
    def display_controller(self) -> DisplayController:
        """
        Returns the robot's display controller.
        """
        return self._display_controller
    @property
    def movement_controller(self) -> MovementController:
        """
        Returns the robot's movement controller.
        """
        return self._movement_controller
    @property
    def audio_controller(self) -> AudioController:
        """
        Returns the robot's audio controller.
        """
        return self._audio_controller