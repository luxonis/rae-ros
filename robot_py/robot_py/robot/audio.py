import os
import random
from ament_index_python import get_package_share_directory

from rae_msgs.srv import PlayAudio


class AudioController:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.audio_client = self.ros_manager.create_service_client(
            '/play_audio', PlayAudio)
        self.assets_path = os.path.join(
            get_package_share_directory('robot_py'), 'assets')
        print("Audio Controller ready")

    def create_and_send_request(self, audio_file_path):
        req = PlayAudio.Request()
        req.mp3_file = audio_file_path
        res = self.ros_manager.call_async_srv('/play_audio', req)
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
