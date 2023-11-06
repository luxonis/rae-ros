#!/usr/bin/env python3
import rclpy
import signal
import sys
from rclpy.node import Node

from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import ColorRGBA
from rae_msgs.msg import LEDControl
from cv_bridge import CvBridge

from PIL import Image as PILImage, ImageDraw
import cv2
import numpy as np


class BatteryStatusNode(Node):

    def __init__(self):
        super().__init__('battery_status_node')
        self.subscription = self.create_subscription(
            BatteryState,
            'battery_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_image = self.create_publisher(Image, 'lcd', 10)
        self.publisher_led = self.create_publisher(LEDControl, 'leds', 10)

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Get battery information from message
        percent = msg.capacity  # assuming percentage is a ratio
        status = msg.power_supply_status

        # Define image size, bar, and padding properties
        width, height = 160, 80
        bars = 10
        padding = 10  # padding at each side of the image

        # Calculate the total width for the bars area (excluding the padding)
        bars_area_width = width - 2 * padding

        # Calculate the bar width and the spacing
        # The bars and the spaces between them fill the bars area, and there are (bars-1) spaces
        bar_width = bars_area_width // (2 * bars - 1)
        spacing = bar_width

        # Create new image with white background
        img = PILImage.new('RGB', (width, height), "black")
        d = ImageDraw.Draw(img)

        # Determine color based on battery percentage
        if percent > 50 and status == 2:
            color = 'green'  # Battery level above 20%
        elif percent > 20 and status == 1:
            color = 'blue'
        elif percent > 10 and status == 2:
            color = 'yellow'  # Battery level between 10% and 20%
        else:
            color = 'red'  # Battery level below 10%

        # Adjust bar height considering padding
        bar_height = height - 2 * padding

        # Draw filled bars proportional to battery level
        for i in range(int(percent // 10)):
            x1 = padding + i * (bar_width + spacing)
            y1 = padding
            x2 = x1 + bar_width
            y2 = padding + bar_height
            d.rectangle([(x1, y1), (x2, y2)], fill=color)

        # Convert PIL image to OpenCV image
        img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

        # Convert OpenCV image to ROS image and publish
        img_msg = self.bridge.cv2_to_imgmsg(img_cv, encoding="bgr8")
        self.publisher_image.publish(img_msg)

        # Set LEDs based on battery level
        # Define colors for LEDs
        colors = {
            "green": ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
            "red": ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            "blue": ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        }

        # Calculate how many LEDs should be turned on based on battery percentage
        led_on_count = int(40 * (percent / 100))

        # Create and publish LEDControl message for each LED
        led_msg = LEDControl()
        led_msg.header.stamp = self.get_clock().now().to_msg()
        led_msg.data = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)]*40
        for i in range(39):
            led_msg.single_led_n = 0
            led_msg.control_type = 2  # assuming 0 means "set color"

            # Turn on the LED if it is within the led_on_count, otherwise turn it off
            if i < led_on_count:
                led_msg.data[i]=(colors[color])

        self.publisher_led.publish(led_msg)


def signal_handler(node):
    def handle(sig, frame):
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        sys.exit(0)
    return handle


def main(args=None):
    rclpy.init(args=args)
    battery_status_node = BatteryStatusNode()
    signal.signal(signal.SIGINT, signal_handler(battery_status_node))
    rclpy.spin(battery_status_node)


if __name__ == '__main__':
    main()
