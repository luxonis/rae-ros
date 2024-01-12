import logging as log
from geometry_msgs.msg import Twist, TransformStamped


class NavigationController:
    """
    A class for controlling the robot's movement.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.

    Methods
    -------
        move(linear, angular): Moves the robot in a given direction.
    
    """

    def __init__(self, ros_interface):
        self._ros_interface = ros_interface
        self._ros_interface.create_publisher("/cmd_vel", Twist)
        log.info("Navigation Controller ready")

    def move(self, linear, angular):
        """
        Move the robot in a given direction.

        Args:
        ----
            linear (float): The linear velocity.
            angular (float): The angular velocity.

        """
        twist_msg = Twist()
        twist_msg.linear.x = float(linear)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular)
        self._ros_interface.publish('/cmd_vel', twist_msg)

    def get_odom_position(self) -> TransformStamped:
        """
        Get the robot's current position relative to odom frame. Returns None if the robot's position is not available.

        Returns
        -------
            TransformStamped: The robot's current position.

        """
        if self._ros_interface.get_frame_position('odom', 'base_footprint') is None:
            return None
        return self._ros_interface.get_frame_position('odom', 'base_footprint')
