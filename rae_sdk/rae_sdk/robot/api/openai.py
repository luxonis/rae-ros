import cv2
import numpy as np

from openai import OpenAI
from pathlib import Path
import base64


class OpenAIClient:
    def __init__(self, key=""):
        self.client = OpenAI(api_key=key)
        self._description = {"role": "system",
                             "content": "Assume you are a mobile robot that has a camera, speakers, microphone and LCD display. Let's say your main goal is to learn how to survive and interact with the world without causing harm to others."
                             }

    @property
    def description(self):
        return self._description

    def generate_speech(self, text: str) -> str:
        """
        Generate speech from text using OpenAI's TTS model

        Args:
        ----
            text (str): The text to generate speech from

        Returns:
        -------
            str: The path to the generated speech file

        """
        speech_file_path = Path(__file__).parent / "speech.mp3"
        response = self.client.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text
        )
        response.stream_to_file(speech_file_path)
        return str(speech_file_path.absolute())

    def _encode_image(self, image) -> bytes:

        cv2.imwrite("./screen1.jpg", image)
        _, buffer = cv2.imencode(".jpg", image)
        return base64.b64encode(buffer).decode('utf-8')

    def describe_image(self, image: np.ndarray, query="What's in this image?") -> str:
        """
        Describe an image using OpenAI's Vision model

        Args:
        ----
            image (np.ndarray): The image to describe
            query (str): The query to ask the model. Default is "What's in this image?"

        Returns:
        -------    
            str: The description of the image
        """
        # OpenAI model to use for the request
        model = "gpt-4-vision-preview"

        # Maximum amount of tokens you want to use for this request
        max_tokens = 300
        base64_image = self.encode_image(image)
        # List of message objects that contains the actual queries
        messages = [
            {
                "role": "user",
                "content": [
                        {
                            "type": "text",
                            "text": query,
                        },
                    {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}",
                            },
                        },
                ],
            }
        ]
        response = self.client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=max_tokens,
        )
        return response

    def respond(self, text: str) -> str:
        """

        Generate a response from the robot using OpenAI's Chat model

        Args:
        ----
            text (str): The text to generate a response from

            Returns:
            -------
            str: The response from the robot

        """
        # OpenAI model to use for the request
        model = "gpt-4"

        # Maximum amount of tokens you want to use for this request
        max_tokens = 300

        # List of message objects that contains the actual queries
        messages = [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": text,
                    }
                ],
            },
            self._description
        ]

        response = self.client.chat.completions.create(
            model=model,
            messages=messages,
            max_tokens=max_tokens,
        )
        return response
