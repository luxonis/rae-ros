from typing import Optional, List, Tuple

from .api.ros.ros2_manager import ROS2Manager
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
        logger: An instance used for logging messages and errors.
        ros_manager (ROS2Manager): An object for managing ROS2 communications and functionalities.
        battery_state (BatteryState): Stores the current state of the robot's battery.
        led_controller (LEDController): Controls the robot's LEDs.
        display_controller (DisplayController): Manages the robot's display.
        movement_controller (MovementController): Handles the robot's movement.

    Methods:
        battery_state_cb(data): Callback method for updating battery state.
        start(): Initializes the robot's components and starts ROS2 communications.
        stop(): Stops the ROS2 communications and shuts down the robot's components.
        move(velocity): Commands the robot to move at a specified velocity.
        display_face(image_data): Displays an image on the robot's face.
        set_leds(led_data): Sets the LED configuration on the robot.
        get_battery(): Retrieves the current battery state.
    """

    def __init__(self):
        """
        Initializes the Robot instance.
        """
        self.ros_manager = None
        self.battery_state = None
        self.led_controller = None
        self.display_controller = None
        self.movement_controller = None
        self.audio_controller = None
        self.perception_system = PerceptionSystem()

    def battery_state_cb(self, data):
        self.battery_state = data

    def start(self, start_hardware=False):
        """
        Initializes and starts the robot's components and ROS2 communications.
        Sets up necessary controllers and subscribers for the robot's functionalities.
        """
        self.ros_manager = ROS2Manager("base_container")
        self.ros_manager.start(start_hardware)
        self.led_controller = LEDController(self.ros_manager)
        self.display_controller = DisplayController(self.ros_manager)
        self.movement_controller = MovementController(self.ros_manager)
        self.audio_controller = AudioController(self.ros_manager)
        self.ros_manager.create_subscriber(
            "/battery_status", BatteryState, self.battery_state_cb)
        self.perception_system.start_ros()

    def stop(self):
        """
        Stops the ROS2 communications and deactivates the robot's controllers.
        Ensures a clean shutdown of all components.
        """
        self.display_controller.stop()
        self.ros_manager.stop()
        self.perception_system.stop()

    def move(self, linear, angular):
        """
        Commands the robot to move at the specified velocity.

        Args:
            linear: Linear velocity.
            angular: Angular velocity.
        """
        self.movement_controller.move(linear, angular)

    def display_face(self, payload):
        """
        Displays face based on given payload.

        Args:
            payload: Data representing the image to be displayed.
        """

        self.display_controller.display_face(payload)

    def display_image(self, image):
        """
        Displays given image in OpenCV format

        Args:
            image: An OpenCV mat
        """
        self.display_controller.display_image(image)

    def display_imu_data(self, imu_data):
        """
        Displays IMU data as 3D axes

        Args:
            imu_data: dai.IMUData
        """
        self.display_controller.display_imu_data(imu_data)

    def set_leds(self, led_data):
        """
        Sets the robot's LEDs based on the provided LED configuration.

        Args:
            led_data: Data or instructions to set the LED configuration.
        """
        self.led_controller.set_leds(led_data)

    def get_battery(self):
        """
        Retrieves the current state of the robot's battery.

        Returns:
            BatteryState: The current state of the battery.
        """
        return self.battery_state

    def honk(self):
        """
        Plays the robot's horn.
        """
        self.audio_controller.honk()

    def display_animation(self):
        """
        Displays the robot's test animation.
        """
        self.display_controller.display_animation()

    def play_random_sfx(self):
        """
        Plays a random sound effect.
        """
        self.audio_controller.play_random_sfx()
    def play_audio(self, audio_path):
        """
        Plays audio from mp3 file.

        Args:
            audio_path: Path to mp3 file
        """
        self.audio_controller.play_audio(audio_path)
    def get_perception_system(self):
        """
        Returns the robot's perception system.
        """
        return self.perception_system

    def get_ros_interface(self):
        """
        Returns the robot's ROS2 interface manager.
        """
        return self.ros_manager
