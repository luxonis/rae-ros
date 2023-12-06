from rae_msgs.msg import LEDControl
from std_msgs.msg import ColorRGBA


class LEDController:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager
        self.ros_manager.create_publisher("/leds", LEDControl)
        print("LED Controller ready")

    def set_leds(self, payload):
        def hex_to_rgb(hex):
            value = hex.lstrip('#')
            lv = len(value)
            return tuple(int(value[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))

        def normalize(num):
            if num == 0:
                return float(num)
            else:
                return float(num)/255.0
        led_msg = LEDControl()

        r, g, b = hex_to_rgb(payload['color'])

        color_msg = ColorRGBA()
        color_msg.a = 1.0
        color_msg.r = normalize(r)
        color_msg.g = normalize(g)
        color_msg.b = normalize(b)
        led_msg.data = [color_msg]
        led_msg.control_type = led_msg.CTRL_TYPE_ALL
        self.ros_manager.publish("/leds", led_msg)
