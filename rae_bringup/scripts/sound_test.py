#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from audio_msgs.msg import Audio
from std_msgs.msg import Header
import time

# Frequencies of the C Major scale (4th octave)
FREQUENCIES = {
    'C4': 261.63,
    'D4': 293.66,
    'E4': 329.63,
    'F4': 349.23,
    'G4': 392.00,
    'A4': 440.00,
    'B4': 493.88,
    'C5': 523.25  # Adding C of the 5th octave
}

class AudioPublisher(Node):

    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(Audio, 'audio_out', 10)
        
        self.sample_rate = 44100  # Sample rate in Hz
        self.duration = 1.0  # Duration of the audio in seconds
        self.t = np.arange(int(self.sample_rate * self.duration)) / self.sample_rate  # Time array
        self.total_frames = 0  # Total number of frames generated so far

        # Define a simple melody (sequence of notes)
        self.melody = ['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']

        # Create the audio signal for the entire melody
        self.melody_signal = self.create_melody_signal()

        # Start publishing the melody
        self.publish_melody()

    def create_melody_signal(self):
        melody_signal = np.array([], dtype=np.int16)
        for note in self.melody:
            frequency = FREQUENCIES[note]
            signal = 0.5 * np.sin(2 * np.pi * frequency * self.t)
            stereo_signal = np.repeat(signal[:, np.newaxis], 2, axis=1)
            stereo_signal_pcm = np.int16(stereo_signal * 32767)
            melody_signal = np.concatenate((melody_signal, stereo_signal_pcm.flatten()))
        return melody_signal.tobytes()

    def publish_melody(self):
        msg = Audio()

        # Fill the Audio message fields
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "audio_frame"
        msg.seq_num = self.total_frames
        msg.frames = len(self.melody_signal) // 4
        msg.channels = 2  # Stereo
        msg.sample_rate = self.sample_rate
        msg.encoding = '16SC2'  # Signed 16-bit stereo audio
        msg.is_bigendian = 0  # Little endian
        msg.layout = Audio.LAYOUT_INTERLEAVED
        msg.step = 4  # 2 channels * 16 bits
        msg.data = self.melody_signal

        while rclpy.ok():
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing an Audio message')
            self.total_frames += msg.frames
            time.sleep(self.duration * len(self.melody))  # Sleep for the duration of the whole melody

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
