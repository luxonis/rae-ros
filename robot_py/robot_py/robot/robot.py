from typing import Optional, List, Tuple

from .api.ros.ros_interface import ROSInterface
from .display import DisplayController
from .led import LEDController
from .movement import MovementController
from .audio import AudioController
from sensor_msgs.msg import BatteryState
from .perception.perception_system import PerceptionSystem


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
        get_battery(): Retrieves the current battery state.
    """

    def __init__(self):
        """
        Initializes the Robot instance.
        """
        self._ros_interface = None
        self._battery_state = None
        self._led_controller = None
        self._display_controller = None
        self._movement_controller = None
        self._audio_controller = None
        self._perception_system = PerceptionSystem()

    def battery_state_cb(self, data):
        self.battery_state = data

    def start(self, start_hardware=False):
        """
        Initializes and starts the robot's components and ROS2 communications.
        Sets up necessary controllers and subscribers for the robot's functionalities.
        """
        self.ros_interface = ROSInterface("base_container")
        self.ros_interface.start(start_hardware)
        self.led_controller = LEDController(self.ros_interface)
        self.display_controller = DisplayController(self.ros_interface)
        self.movement_controller = MovementController(self.ros_interface)
        self.audio_controller = AudioController(self.ros_interface)
        self.ros_interface.create_subscriber(
            "/battery_status", BatteryState, self.battery_state_cb)
        self.perception_system.setup_perception_rtabmap()

    def stop(self):
        """
        Stops the ROS2 communications and deactivates the robot's controllers.
        Ensures a clean shutdown of all components.
        """
        self.display_controller.stop()
        self.ros_interface.stop()
        self.perception_system.stop()

    @property
    def get_battery(self):
        """
        Retrieves the current state of the robot's battery.

        Returns:
            BatteryState: The current state of the battery.
        """
        return self.battery_state
    @property
    def perception_system(self) -> PerceptionSystem:
        """
        Returns the robot's perception system.
        """
        return self.perception_system
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
