# This message contains LED data for a single LED or a group of LEDs

std_msgs/Header header  
rae_msgs/ColorPeriod[] data
uint8 single_led_n
uint8 control_type
uint8 animation_size
uint8 animation_quantity

uint8 CTRL_TYPE_ALL=0
uint8 CTRL_TYPE_SINGLE=1
uint8 CTRL_TYPE_SPINNER=2
uint8 CTRL_TYPE_FAN=3
uint8 CTRL_TYPE_CUSTOM=4
