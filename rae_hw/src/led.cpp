#include "rae_hw/leds.hpp"

namespace rae_hw
{
    Leds::Leds()
        : Node("led_node"),
         logicalToPhysicalMapping{
            {0, 0}, 
            {1, 1},
            {2, 2}, 
            {3, 3},
            {4, 4}, 
            {5, 5},
            {6, 6}, 
            {7, 7},
            {8, 8}, 
            {9, 29},
            {10, 30}, 
            {11, 31},
            {12, 32}, 
            {13, 33},
            {14, 34}, 
            {15, 35},
            {16, 36}, 
            {17, 37},
            {18, 38}, 
            {19, 19},
            {20, 20}, 
            {21, 21},
            {22, 22},
            {23, 23}, 
            {24, 24},
            {25, 25}, 
            {26, 26},
            {27, 27}, 
            {28, 28},
            {29, 9}, 
            {30, 10},
            {31, 11},
            {32, 12}, 
            {33, 13},
            {34, 14}, 
            {35, 15},
            {36, 16}, 
            {37, 17},
            {38, 18}
        },
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
            "leds", 10, std::bind(&Leds::topic_callback, this, std::placeholders::_1));
        setAllPixels(150, 10, 150);
        transmitSPI();
        RCLCPP_INFO(this->get_logger(), "LED node running!");

       
    }
    Leds::~Leds(){
        setAllPixels(0, 0, 0);
        transmitSPI();
    }
    void Leds::topic_callback(const rae_msgs::msg::LEDControl &msg)
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
        uint8_t Leds::convertColor(float num)
    {
        return static_cast<uint8_t>(round(num * 255.0));
    }
    void Leds::transmitSPI()
    {
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)ws2812b_buffer,
            .len = WS2812B_BUFFER_SIZE,
            .speed_hz = speed,
            .delay_usecs = 0,
            .bits_per_word = 8,
        };
        int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

        if (ret < 1)
            RCLCPP_ERROR(this->get_logger(), "can't send spi message");
    }
    void Leds::fillBuffer(uint8_t color)
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
    void Leds::setSinglePixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b)
    {   auto mapping_pair = logicalToPhysicalMapping.find(pixel);
        if (mapping_pair != logicalToPhysicalMapping.end())
        {
            uint16_t physicalID = mapping_pair->second;
            ptr = &ws2812b_buffer[24 * physicalID];
        }
        else
        {
            // Handle invalid logical IDs or cases not covered in the mapping
            // For example, print a message indicating the LED doesn't exist
            std::cout << "Logical LED ID " << pixel << " doesn't have a corresponding physical LED." << std::endl;
            // You can also choose to skip or set a default pixel color here
        }
        
        fillBuffer(g);
        fillBuffer(r);
        fillBuffer(b);
    }

    void Leds::setAllPixels(uint8_t r, uint8_t g, uint8_t b)
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
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rae_hw::Leds>());
    rclcpp::shutdown();
    return 0;
}