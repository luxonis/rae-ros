import os
import logging as log
from copy import deepcopy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory
from .state import StateInfo


def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a rotation matrix.

    Arguments
    -----------
    q (tuple): A quaternion represented as (q_w, q_x, q_y, q_z).

    Returns
    -------
    numpy.ndarray: A 3x3 rotation matrix.

    """
    q_w, q_x, q_y, q_z = q
    sq_w, sq_x, sq_y, sq_z = q_w ** 2, q_x ** 2, q_y ** 2, q_z ** 2

    # Rotation matrix
    rot_matrix = np.array([
        [1 - 2 * sq_y - 2 * sq_z,     2 * q_x * q_y - 2 *
            q_z * q_w,     2 * q_x * q_z + 2 * q_y * q_w],
        [2 * q_x * q_y + 2 * q_z * q_w, 1 - 2 * sq_x -
            2 * sq_z,     2 * q_y * q_z - 2 * q_x * q_w],
        [2 * q_x * q_z - 2 * q_y * q_w, 2 * q_y *
            q_z + 2 * q_x * q_w, 1 - 2 * sq_x - 2 * sq_y]
    ])

    return rot_matrix


class DisplayController:
    """
    A class for controlling the robot's display.

    Attributes
    ----------
        _ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        _bridge (CvBridge): An object for converting between ROS2 and OpenCV image formats.
        _screen_width (int): The width of the robot's display.
        _screen_height (int): The height of the robot's display.
        _assets_path (str): The path to the robot's assets directory.

    Methods
    -------
        stop(): Stops the display.
        display_default(): Displays the default image on the robot's display.
        display_face(payload): Displays a face on the robot's display.
        display_image(image_data): Displays an image on the robot's display.
        display_imu_data(imu_data): Displays IMU data on the robot's display.
        display_animation(): Displays an animation on the robot's display.
        ball_callback(): Callback method for displaying an animation on the robot's display.

    """

    def __init__(self, ros_interface):
        self._ros_interface = ros_interface
        self._ros_interface.create_publisher('/lcd', Image)
        self._bridge = CvBridge()
        self._screen_width = 160
        self._screen_height = 80
        self._assets_path = os.path.join(
            get_package_share_directory('rae_sdk'), 'assets')
        self._default_img = cv2.imread(os.path.join(self._assets_path, 'img', 'rae-logo-white.jpg'))
        self._last_image = None
        self._state_info = None
        self.display_default()
        log.info("Display Controller ready")

    def stop(self):
        self.display_default()

    def display_image(self, image_data):
        self._last_image = image_data
        if self._state_info:
            overlay = self.battery_overlay()
            image_data = cv2.addWeighted(image_data, 1, overlay, 1, 0)
        ros_image = self._bridge.cv2_to_imgmsg(image_data, encoding='bgra8')
        self._ros_interface.publish('/lcd', ros_image)

    def add_state_overlay(self, info: StateInfo):
        self._state_info = info
        self.display_image(deepcopy(self._last_image))
    
    def display_text(self, text, on_default=True, location=(30,16), color=(255, 255, 255), font_scale=0.5, thickness=1, font=cv2.FONT_HERSHEY_SIMPLEX, line_type=cv2.LINE_AA):
        img = np.zeros(
                (self._screen_height, self._screen_width, 3), dtype=np.uint8)
        if on_default:
            self.display_default()
            img = deepcopy(self._last_image)
            
        cv2.putText(img, text, location, font, font_scale, color, thickness, line_type)
        bgra_image = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        self.display_image(bgra_image)
    
    def battery_overlay(self):
        # display battery state in a rectangle on the top right corner of the screen
        battery_state = self._state_info.battery_state
        img = self._last_image
        # create battery symbol
        cv2.rectangle(img, (140, 5), (156, 15), (255, 255, 255), 1)
        cv2.rectangle(img, (156, 7), (158, 13), (255, 255, 255), -1)
        # create 3 bars based on battery percentage, if above 66% color is green, 66-33% is yellow, below 33% is red
        if battery_state.capacity > 66:
            color = (0, 255, 0)
        elif battery_state.capacity > 33:
            color = (0, 255, 255)
        else:
            color = (0, 0, 255)
        cv2.rectangle(img, (142, 7), (143 + int(battery_state.capacity / 10), 13), color, -1)
        # fill the rest with black
        cv2.rectangle(img, (143 + int(battery_state.capacity / 10), 7), (156, 13), (0, 0, 0), -1)
        bgra_image = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
        return bgra_image

    def display_default(self):
        bgra_image = cv2.cvtColor(self._default_img, cv2.COLOR_BGR2BGRA)
        self.display_image(bgra_image)

    def display_face(self, payload):
        img_name = payload[1]['name']
        image_path = os.path.join(self._assets_path, 'img', f'{img_name}.png')

        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

        def hex_to_rgb(hex):
            value = hex.lstrip('#')
            lv = len(value)
            return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))
        r, g, b = hex_to_rgb(payload[0]['color'])

        # Check if the image has an alpha channel

        if image.shape[2] == 4:

            # Split the image into its channels
            b_channel, g_channel, r_channel, alpha_channel = cv2.split(image)
            # Define the color to replace the transparent parts (purple in this case)
            # OpenCV uses BGR format, so purple is (128, 0, 128)
            replacement_color = [b, g, r]
            # Find all pixels where the alpha channel is zero
            transparent_mask = alpha_channel < 150
            # Set the color of these pixels to the replacement color
            image[transparent_mask] = replacement_color + \
                [255]  # Add full opacity to the color
            self.display_image(image)

    def display_imu_data(self, imu_data):
        axis_length = 20

        packets = imu_data.packets
        for packet in packets:
            rotation = packet.rotationVector
            # Get rotation matrix
            rotation_matrix = quaternion_to_rotation_matrix(
                (rotation.i, rotation.j, rotation.k, rotation.real))

            # Define 3D axes
            axes = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [
                              0, 0, axis_length], [0, 0, 0]]).reshape(-1, 3)

            # Project 3D points to 2D
            focal_length = self._screen_width
            camera_matrix = np.array([[focal_length, 0, self._screen_width / 2],
                                      [0, focal_length, self.screen_height / 2],
                                      [0, 0, 1]], dtype='double')
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            axes_2d, _ = cv2.projectPoints(
                axes, rotation_matrix, np.zeros((3, 1)), camera_matrix, dist_coeffs)

            # Create image and draw the axes
            image = np.zeros(
                (self.screen_height, self._screen_width, 3), dtype=np.uint8)
            axes_2d = np.int32(axes_2d).reshape(-1, 2)
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[0]), (0, 0, 255), 3)  # X-axis in red
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[1]), (0, 255, 0), 3)  # Y-axis in green
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[2]), (255, 0, 0), 3)  # Z-axis in blue
            bgra_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            self.display_image(bgra_image)

    def display_animation(self, rate, callback):
        self._ros_interface.create_timer('animation', rate, callback)

    def ball_callback(self):
        # Ball properties
        self.ball_size = 20
        self.ball_color = (255, 0, 255)  # Red in BGR format

        # Ball initial position and velocity
        self.x, self.y = self._screen_width // 2, self.screen_height // 2
        self.vx, self.vy = 2, 3  # Speed of the ball in x and y direction
        image = np.zeros(
            (self.screen_height, self._screen_width, 3), dtype=np.uint8)

        # Draw the ball
        cv2.circle(image, (self.x, self.y),
                   self.ball_size // 2, self.ball_color, -1)

        # Update the ball's position
        self.x += self.vx
        self.y += self.vy

        # Bounce off the walls
        if self.x <= self.ball_size // 2 or self.x >= self._screen_width - self.ball_size // 2:
            self.vx = -self.vx
        if self.y <= self.ball_size // 2 or self.y >= self.screen_height - self.ball_size // 2:
            self.vy = -self.vy
        bgra_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
        self.display_image(bgra_image)
