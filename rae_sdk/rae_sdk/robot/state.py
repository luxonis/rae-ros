import logging as log
from sensor_msgs.msg import BatteryState

class StateController:
    """
    A class for managing the robot's state.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        battery_state (BatteryState): Stores the current state of the robot's battery.

    Methods
    -------
        battery_state_cb(data): Callback method for updating battery state.
        start(): Initializes the robot's components and starts ROS2 communications.
        stop(): Stops the ROS2 communications and shuts down the robot's components.

    """

    def __init__(self, ros_interface):
        self._ros_interface = ros_interface
        self._ros_interface.create_subscriber(
            "/battery_status", BatteryState, self.battery_state_cb)
        self._battery_state = None
        log.info("State Controller ready")

    def battery_state_cb(self, data):
        self._battery_state = data

    @property
    def battery_state(self):
        return self._battery_state