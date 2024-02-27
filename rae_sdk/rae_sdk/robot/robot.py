from typing import Optional, List, Tuple
import logging as log
from .api.ros.ros_interface import ROSInterface
from .display import DisplayController
from .led import LEDController
from .navigation import NavigationController
from .audio import AudioController
from .state import StateController
from .perception.perception_system import PerceptionSystem
from .robot_options import RobotOptions


class Robot:
    """
    A class representing a robot, integrating various controllers for movement, display, and LED management and interfacing with ROS2 for communication and control.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        led (LEDController): Controls the robot's LEDs.
        display (DisplayController): Manages the robot's display.
        navigation (NavigationController): Handles the robot's movement.
        audio (AudioController): Controls the robot's audio.
        state (StateController): Manages the robot's state information.
        perception (PerceptionSystem): Handles the robot's perception system.

    Methods
    -------
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
            self._navigation_controller = NavigationController(
                self._ros_interface)
            self._audio_controller = AudioController(self._ros_interface)
            self._state_controller = StateController(
                self._ros_interface, robot_options.publish_state_info, self._display_controller)
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

    @property
    def state(self) -> StateController:
        return self._state_controller

    @property
    def perception(self) -> PerceptionSystem:
        """Create perception system if it doesn't exist and return it."""
        if self._perception_system is None:
            self._perception_system = PerceptionSystem(
                self._robot_options.namespace)
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
    def navigation(self) -> NavigationController:
        return self._navigation_controller

    @property
    def audio(self) -> AudioController:
        return self._audio_controller
