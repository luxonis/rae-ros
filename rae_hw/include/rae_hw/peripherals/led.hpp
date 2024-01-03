#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <map>

#include "rae_hw/peripherals/spidev.h"
#include "rae_msgs/msg/led_control.hpp"
#include "rclcpp/rclcpp.hpp"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define WS2812B_NUM_LEDS 39
#define WS2812B_RESET_PULSE 60
#define WS2812B_BUFFER_SIZE (WS2812B_NUM_LEDS * 24 + WS2812B_RESET_PULSE)

namespace rae_hw {
class LEDNode : public rclcpp::Node {
   public:
    LEDNode(const rclcpp::NodeOptions& options);
    ~LEDNode();

   private:
    std::map<uint16_t, uint16_t> logicalToPhysicalMapping;
    void transmitSPI();
    void fillBuffer(uint8_t color);
    uint8_t convertColor(float num);
    void setSinglePixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);
    void setAllPixels(uint8_t r, uint8_t g, uint8_t b);
    void topic_callback(const rae_msgs::msg::LEDControl& msg);
    rclcpp::Subscription<rae_msgs::msg::LEDControl>::SharedPtr subscription_;
    uint8_t* ptr;
    uint32_t mode;
    // static uint32_t speed = 1316134912;
    uint32_t speed = 3200000;
    uint8_t bits = 8;
    int fd = 0;
    uint8_t ws2812b_buffer[WS2812B_BUFFER_SIZE];
};
}  // namespace rae_hw
