from geometry_msgs.msg import Twist


class MovementController:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.ros_manager.create_publisher("/cmd_vel", Twist)
        print("Movement Controller ready")

    def move(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = float(linear)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular)
        self.ros_manager.publish('/cmd_vel', twist_msg)
