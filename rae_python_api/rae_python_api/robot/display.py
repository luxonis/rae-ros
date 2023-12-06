import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory


def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a rotation matrix.

    Parameters:
    q (tuple): A quaternion represented as (q_w, q_x, q_y, q_z).

    Returns:
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
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.ros_manager.create_publisher('/lcd', Image)
        self.bridge = CvBridge()
        self.screen_width = 160
        self.screen_height = 80
        self.assets_path = os.path.join(
            get_package_share_directory('rae_python_api'), 'assets')
        print("Display Controller ready")

    def stop(self):
        self.display_default()

    def display_default(self):
        path = os.path.join(self.assets_path, 'img', 'rae-logo-white.jpg')
        image = cv2.imread(path)
        bgra_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
        ros_image = self.bridge.cv2_to_imgmsg(bgra_image, encoding='bgra8')
        self.ros_manager.publish('/lcd', ros_image)

    def display_face(self, payload):
        img_name = payload[1]['name']
        image_path = f'/app/src/robot/assets/img/{img_name}.png'

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
            replacement_color = [r, g, b]
            # Find all pixels where the alpha channel is zero
            transparent_mask = alpha_channel < 150
            # Set the color of these pixels to the replacement color
            image[transparent_mask] = replacement_color + \
                [255]  # Add full opacity to the color
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgra8')
            self.ros_manager.publish('/lcd', ros_image)

    def display_image(self, image_data):
        ros_image = self.bridge.cv2_to_imgmsg(image_data, encoding='bgra8')
        self.ros_manager.publish('/lcd', ros_image)

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
            focal_length = self.screen_width
            camera_matrix = np.array([[focal_length, 0, self.screen_width / 2],
                                      [0, focal_length, self.screen_height / 2],
                                      [0, 0, 1]], dtype='double')
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            axes_2d, _ = cv2.projectPoints(
                axes, rotation_matrix, np.zeros((3, 1)), camera_matrix, dist_coeffs)

            # Create image and draw the axes
            image = np.zeros(
                (self.screen_height, self.screen_width, 3), dtype=np.uint8)
            axes_2d = np.int32(axes_2d).reshape(-1, 2)
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[0]), (0, 0, 255), 3)  # X-axis in red
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[1]), (0, 255, 0), 3)  # Y-axis in green
            cv2.line(image, tuple(axes_2d[3]), tuple(
                axes_2d[2]), (255, 0, 0), 3)  # Z-axis in blue
            bgra_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
            ros_image = self.bridge.cv2_to_imgmsg(bgra_image, encoding='bgra8')
            self.ros_manager.publish('/lcd', ros_image)

    def display_animation(self):
        # Ball properties
        self.ball_size = 20
        self.ball_color = (255, 0, 255)  # Red in BGR format

        # Ball initial position and velocity
        self.x, self.y = self.screen_width // 2, self.screen_height // 2
        self.vx, self.vy = 2, 3  # Speed of the ball in x and y direction
        self.ros_manager.create_timer('animation', 0.05, self.ball_callback)

    def ball_callback(self):

        image = np.zeros(
            (self.screen_height, self.screen_width, 3), dtype=np.uint8)

        # Draw the ball
        cv2.circle(image, (self.x, self.y),
                   self.ball_size // 2, self.ball_color, -1)

        # Update the ball's position
        self.x += self.vx
        self.y += self.vy

        # Bounce off the walls
        if self.x <= self.ball_size // 2 or self.x >= self.screen_width - self.ball_size // 2:
            self.vx = -self.vx
        if self.y <= self.ball_size // 2 or self.y >= self.screen_height - self.ball_size // 2:
            self.vy = -self.vy
        bgra_image = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
        ros_image = self.bridge.cv2_to_imgmsg(bgra_image, encoding='bgra8')
        self.ros_manager.publish('/lcd', ros_image)
