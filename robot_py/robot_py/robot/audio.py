import os
import random
import logging as log
from ament_index_python import get_package_share_directory

from rae_msgs.srv import PlayAudio


class AudioController:
    """
    A class for controlling the robot's audio.

    Attributes:
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.
        audio_client (Client): A ROS2 client for playing audio.
        assets_path (str): The path to the robot's assets directory.

    Methods:
        create_and_send_request(audio_file_path): Creates and sends a request to play an audio file.
        play_audio_file(audio_file_path): Plays an audio file.
        honk(): Plays a horn sound.
        play_random_sfx(): Plays a random sound effect.
    """
    def __init__(self, ros_interface):
        self.ros_interface = ros_interface
        self.audio_client = self.ros_interface.create_service_client(
            'play_audio', PlayAudio)
        self.assets_path = os.path.join(
            get_package_share_directory('robot_py'), 'assets')
        log.info("Audio Controller ready")

    def create_and_send_request(self, audio_file_path):
        req = PlayAudio.Request()
        req.mp3_file = audio_file_path
        res = self.ros_interface.call_async_srv('play_audio', req)
        return res

    def play_audio_file(self, audio_file_path):
        res = self.create_and_send_request(audio_file_path)

    def honk(self):
        horn_path = os.path.join(self.assets_path, 'sfx', 'horn.mp3')
        res = self.create_and_send_request(horn_path)

    def play_random_sfx(self):
        random_sfx_path = os.path.join(self.assets_path, 'sfx', 'voices')
        file = random.choice(os.listdir(random_sfx_path))
        file_path = os.path.join(random_sfx_path, file)
        res = self.create_and_send_request(file_path)
