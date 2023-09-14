#include "rae_hw/peripherals/led.hpp"

namespace rae_hw
{
    LEDNode::LEDNode(const rclcpp::NodeOptions &options)
        : Node("led_node", options)
    {
        int ret = 0;
        memset(ws2812b_buffer, 0, WS2812B_BUFFER_SIZE);
        fd = open("/dev/spidev3.0", O_RDWR);
        if (fd < 0)
            RCLCPP_ERROR(this->get_logger(), "can't open device");

        ret = ioctl(fd, SPI_IOC_WR_MODE32, &mode);
        if (ret == -1)
            RCLCPP_ERROR(this->get_logger(), "can't set spi mode");

        /*
         * bits per word
         */
        ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        if (ret == -1)
            RCLCPP_ERROR(this->get_logger(), "can't set bits per word");
        /*
         * max speed hz
         */
        ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1)
            RCLCPP_ERROR(this->get_logger(), "can't set max speed hz");

        for (int i = 0; i < WS2812B_BUFFER_SIZE; i++)
        {
            ws2812b_buffer[i] = 128;
        }

        subscription_ = this->create_subscription<rae_msgs::msg::LEDControl>(
            "led", 10, std::bind(&LEDNode::topic_callback, this, std::placeholders::_1));
        setAllPixels(150, 10, 150);
        transmitSPI();
        RCLCPP_INFO(this->get_logger(), "LED node running!");
    }
    LEDNode::~LEDNode()
    {
        setAllPixels(0, 0, 0);
        transmitSPI();
    }
    void LEDNode::topic_callback(const rae_msgs::msg::LEDControl &msg)
    {
        if (msg.control_type == msg.CTRL_TYPE_ALL)
        {
            uint8_t r = convertColor(msg.data[0].r);
            uint8_t g = convertColor(msg.data[0].g);
            uint8_t b = convertColor(msg.data[0].b);
            setAllPixels(r, g, b);
        }
        else if (msg.control_type == msg.CTRL_TYPE_SINGLE)
        {
            uint8_t r = convertColor(msg.data[0].r);
            uint8_t g = convertColor(msg.data[0].g);
            uint8_t b = convertColor(msg.data[0].b);
            setSinglePixel(msg.single_led_n, r, g, b);
        }
        else
        {
            for (int i = 0; i < WS2812B_NUM_LEDS; i++)
            {
                uint8_t r = convertColor(msg.data[i].r);
                uint8_t g = convertColor(msg.data[i].g);
                uint8_t b = convertColor(msg.data[i].b);
                setSinglePixel(i, r, g, b);
            }
        }
        transmitSPI();
    }
    uint8_t LEDNode::convertColor(float num)
    {
        return static_cast<uint8_t>(round(num * 255.0));
    }
    void LEDNode::transmitSPI()
    {
        struct spi_ioc_transfer tr = spi_ioc_transfer();
        tr.tx_buf = (unsigned long)ws2812b_buffer;
        tr.len = WS2812B_BUFFER_SIZE;
        tr.speed_hz = speed;
        tr.delay_usecs = 0;
        tr.bits_per_word = 8;
        
        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

        if (ret < 1)
            RCLCPP_ERROR(this->get_logger(), "can't send spi message");
    }
    void LEDNode::fillBuffer(uint8_t color)
    {
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            if (color & mask)
            {
                *ptr = 0xfc;
            }
            else
            {
                *ptr = 0x80;
            }
            ptr++;
        }
    }
    void LEDNode::setSinglePixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b)
    {
        ptr = &ws2812b_buffer[24 * pixel];
        fillBuffer(g);
        fillBuffer(r);
        fillBuffer(b);
    }

    void LEDNode::setAllPixels(uint8_t r, uint8_t g, uint8_t b)
    {
        // printf("All pixels\n");
        ptr = ws2812b_buffer;
        for (uint16_t i = 0; i < WS2812B_NUM_LEDS; ++i)
        {
            fillBuffer(g);
            fillBuffer(r);
            fillBuffer(b);
        }
    }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rae_hw::LEDNode);