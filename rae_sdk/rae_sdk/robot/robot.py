from typing import Optional, List, Tuple
import logging as log
from .api.ros.ros_interface import ROSInterface
from .display import DisplayController
from .led import LEDController
from .movement import MovementController
from .audio import AudioController
from .perception.perception_system import PerceptionSystem
from .robot_options import RobotOptions
from sensor_msgs.msg import BatteryState


class Robot:
    """
    A class representing a robot, integrating various controllers for movement, display, and LED management and interfacing with ROS2 for communication and control.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        battery_state (BatteryState): Stores the current state of the robot's battery.
        led_controller (LEDController): Controls the robot's LEDs.
        display_controller (DisplayController): Manages the robot's display.
        movement_controller (MovementController): Handles the robot's movement.
        audio_controller (AudioController): Controls the robot's audio.
        perception_system (PerceptionSystem): Handles the robot's perception system.

    Methods
    -------
        battery_state_cb(data): Callback method for updating battery state.
        start(): Initializes the robot's components and starts ROS2 communications.
        stop(): Stops the ROS2 communications and shuts down the robot's components.

    """

    def __init__(self, robot_options: RobotOptions = RobotOptions()):
        """
        Initialize the Robot instance.

        Args:
        ----
            robot_options (RobotOptions): An object containing the robot's options.

        """
        self._robot_options = robot_options
        self._ros_interface = ROSInterface(robot_options)
        self._ros_interface.start()
        if robot_options.launch_controllers:
            self._led_controller = LEDController(self._ros_interface)
            self._display_controller = DisplayController(self._ros_interface)
            self._movement_controller = MovementController(self._ros_interface)
            self._audio_controller = AudioController(self._ros_interface)
            self._ros_interface.create_subscriber(
                "/battery_status", BatteryState, self.battery_state_cb)

        self._perception_system = None
        log.info('Robot ready')

    def __del__(self) -> None:
        self.stop()

    def stop(self):
        """
        Stop the ROS2 communications and deactivates the robot's controllers.

        Ensures a clean shutdown of all components.
        """
        if self._perception_system is not None:
            self._perception_system.stop()
        if self._display_controller is not None:
            self._display_controller.stop()
        self._ros_interface.stop()

    def battery_state_cb(self, data):
        self._battery_state = data

    @property
    def battery_state(self) -> BatteryState:
        return self._battery_state

    @property
    def perception_system(self) -> PerceptionSystem:
        """Creates perception system if it doesn't exist and returns it."""
        if self._perception_system is None:
            self._perception_system = PerceptionSystem(self._robot_options.namespace)
        return self._perception_system

    @property
    def ros_interface(self) -> ROSInterface:
        return self._ros_interface

    @property
    def led(self) -> LEDController:
        return self._led_controller

    @property
    def display(self) -> DisplayController:
        return self._display_controller

    @property
    def movement(self) -> MovementController:
        return self._movement_controller

    @property
    def audio(self) -> AudioController:
        return self._audio_controller
