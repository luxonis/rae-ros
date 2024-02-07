import logging as log
from rae_msgs.msg import LEDControl, ColorPeriod
from std_msgs.msg import ColorRGBA


class LEDController:
    """
    A class for controlling the robot's LEDs.

    Attributes
    ----------
        ros_interface (ROSInterface): An object for managing ROS2 communications and functionalities.

    Methods
    -------
        set_leds(payload): Sets the robot's LEDs to a given color.

    """

    def __init__(self, ros_interface):
        self._ros_interface = ros_interface
        self._ros_interface.create_publisher("/leds", LEDControl)
        self._effect_types_map = {
            "all": LEDControl.CTRL_TYPE_ALL,
            "single": LEDControl.CTRL_TYPE_SINGLE,
            "spinner": LEDControl.CTRL_TYPE_SPINNER,
            "fan": LEDControl.CTRL_TYPE_FAN,
            "custom": LEDControl.CTRL_TYPE_CUSTOM,
        }
        log.info("LED Controller ready")

    def hex_to_rgb(self, hex):
        """Convert a hex color to an RGB tuple."""
        value = hex.lstrip('#')
        lv = len(value)
        return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

    def normalize(self, num):
        """Normalize a number to a float between 0 and 1."""
        if num == 0:
            return float(num)
        else:
            return float(num)/255.0

    def set_leds_from_payload(self, payload: dict):
        """
        Set the robot's LEDs to a given color.

        Args:
        ----
            payload (dict): A dictionary containing the color to set the LEDs to. 
            Example payload struct: {'brightness': 50, 'color': '#FFFFFF', 'effect': 'pulse', 'interval': 5}

        """
        led_msg = LEDControl()

        r, g, b = self.hex_to_rgb(payload['color'])
        color_msg = ColorPeriod()
        if 'interval' in payload.keys():
            color_msg.frequency = float(payload['interval'])
        else:
            color_msg.frequency = 0.0
        color_msg.color.a = float(payload['brightness']) / 100
        color_msg.color.r = self.normalize(r)
        color_msg.color.g = self.normalize(g)
        color_msg.color.b = self.normalize(b)
        led_msg.data = [color_msg]
        if payload['effect'] in self._effect_types_map:
            led_msg.control_type = self._effect_types_map[payload['effect']]
        else:
            led_msg.control_type = LEDControl.CTRL_TYPE_ALL
        if 'size' in payload.keys():
            led_msg.animation_size = payload['size']
        if 'blades' in payload.keys():
            led_msg.animation_quantity = payload['blades']
        self._ros_interface.publish("/leds", led_msg)

    def set_leds(self, color: str, brightness: int = 100, effect: str = "solid", interval: int = 5):
        """
        Set the robot's LEDs to a given color.

        Args:
        ----
            color (str): The color to set the LEDs to.
            brightness (int): The brightness of the LEDs. (Default: 100)
            effect (str): The effect to apply to the LEDs. (Default: "solid")
            interval (int): The interval of the effect. (Default: 5)

        """
        payload = {
            'brightness': brightness,
            'color': color,
            'effect': effect,
            'interval': interval
        }
        self.set_leds_from_payload(payload)

    def set_leds_from_msg(self, msg: LEDControl):
        """
        Set the robot's LEDs to a given color.

        Args:
        ----
            msg (LEDControl): The message containing the color to set the LEDs to.

        """
        self._ros_interface.publish("/leds", msg)
