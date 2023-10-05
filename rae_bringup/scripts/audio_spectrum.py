#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from audio_msgs.msg import Audio
from rae_msgs.msg import LEDControl
from std_msgs.msg import Header, ColorRGBA

class AudioVisualizer(Node):
    def __init__(self):
        super().__init__('audio_visualizer')
        self.subscriber = self.create_subscription(
            Audio,
            'audio_in',  # Assuming the topic name is "audio_data"
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Image, 'lcd', 10)
        self.bridge = CvBridge()

        self.led_publisher = self.create_publisher(LEDControl, '/leds', 10)  # Add LEDControl publisher
        self.led_control_message = LEDControl()
        self.led_control_message.control_type = LEDControl.CTRL_TYPE_ALL  # control all LEDs
        self.led_control_message.header = Header()
        self.led_threshold = 0.2  # Adjust this value according to your loudness definition


    def listener_callback(self, msg):
        # Assuming audio data is 32-bit signed little-endian PCM
        assert not msg.is_bigendian
        # Convert bytes to numpy array
        audio_data = None
        if msg.encoding == "S32LE":
            audio_data = np.frombuffer(msg.data, dtype=np.int32)
        elif msg.encoding == "S16LE":
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
        if msg.layout == Audio.LAYOUT_INTERLEAVED:
            # Deinterleave channels
            audio_data = audio_data.reshape((msg.frames, msg.channels))

        # Compute FFT and get magnitude spectrum
        spectrum = np.abs(np.fft.rfft(audio_data, axis=0))

        # Normalize spectrum to 0-1
        spectrum = spectrum / np.max(spectrum)*10

        # Compute the frequencies for each FFT bin
        freqs = np.fft.rfftfreq(msg.frames, 1.0 / msg.sample_rate)

        # Create a 160x80 image
        img = np.zeros((80, 160, 3), dtype=np.uint8)

        # Plot 30 bars for frequencies
        for i in range(30):
            # Determine the frequency range for this bar
            freq_min = i * msg.sample_rate / 2 / 30
            freq_max = (i+1) * msg.sample_rate / 2 / 30

            # Find FFT bins that fall into this range
            mask = (freqs >= freq_min) & (freqs < freq_max)

            # Compute the average magnitude in this range
            mag_avg = np.mean(spectrum[mask])

            # Draw the bar in the image
            cv2.rectangle(img, (i * 5, 80), ((i+1) * 5, 80 - int(mag_avg * 80)), (0, 255, 0), -1)

        # Convert the OpenCV image to a ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")

        # Publish the image

        self.publisher.publish(img_msg)

        # Determine loudness as the average magnitude of the spectrum
        loudness = np.mean(spectrum)
        # self.get_logger().info(f"{loudness}")

        # Turn LEDs on if loudness exceeds a certain threshold, otherwise turn them off
        if loudness > self.led_threshold:
            self.led_control_message.data = [ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)]  # white color
        else:
            self.led_control_message.data = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)]  # black/off color

        # Publish the LEDControl message
        self.led_publisher.publish(self.led_control_message)


def main(args=None):
    rclpy.init(args=args)

    audio_visualizer = AudioVisualizer()

    rclpy.spin(audio_visualizer)

    audio_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()