import logging as log
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature



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

    """

    def __init__(self, ros_interface):
        self._ros_interface = ros_interface

        self._battery_state = BatteryState()
        self._cpu_usage = 0.0
        self._mem_usage = 0.0
        self._temp = 0.0
        self._disk = 0.0
        self._net_up = 0.0
        self._net_down = 0.0
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
        
        log.info("State Controller ready")

    def battery_state_cb(self, data):
        self._battery_state = data
    
    def cpu_usage_cb(self, data):
        self._cpu_usage = data.data

    def mem_usage_cb(self, data):
        self._mem_usage = data.data

    def temp_cb(self, data):
        self._temp = data.temperature

    def disk_cb(self, data):
        self._disk = data.data

    def net_up_cb(self, data):
        self._net_up = data.data

    def net_down_cb(self, data):
        self._net_down = data.data

    @property
    def battery_state(self):
        return self._battery_state
    
    @property
    def cpu_usage(self):
        return self._cpu_usage
    
    @property
    def mem_usage(self):
        return self._mem_usage
    
    @property
    def temp(self):
        return self._temp
    
    @property
    def disk(self):
        return self._disk
    
    @property
    def net_up(self):
        return self._net_up
    
    @property
    def net_down(self):
        return self._net_down
