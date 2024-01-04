#include "rae_hw/peripherals/led.hpp"

namespace rae_hw
{
    LEDNode::LEDNode(const rclcpp::NodeOptions &options)
        : Node("led_node", options),
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
        }
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
            "leds", 10, std::bind(&LEDNode::topic_callback, this, std::placeholders::_1));

        setAllPixels(150, 10, 150, 0);
        transmitSPI();
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LEDNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "LED node running!");
    }
    LEDNode::~LEDNode()
    {
        setAllPixels(0, 0, 0, 0);
        transmitSPI();
    }
    void LEDNode::topic_callback(const rae_msgs::msg::LEDControl::SharedPtr msg) {   
        std::lock_guard<std::mutex> lock(mutex_);
        currentData_ = msg;
        conditionVariable_.notify_one();  // Notify the LED control 
}

    void LEDNode::timer_callback () {

        std::lock_guard<std::mutex> lock(mutex_);
        if (currentData_) {
        
        if (currentData_->control_type == currentData_->CTRL_TYPE_ALL)
        {
            uint8_t r = convertColor(currentData_->data[0].r);
            uint8_t g = convertColor(currentData_->data[0].g);
            uint8_t b = convertColor(currentData_->data[0].b);
            setAllPixels(r, g, b, currentData_-> data[0].f);
        }
        else if (currentData_->control_type == currentData_->CTRL_TYPE_SINGLE)
        {
            uint8_t r = convertColor(currentData_->data[0].r);
            uint8_t g = convertColor(currentData_->data[0].g);
            uint8_t b = convertColor(currentData_->data[0].b);
            setSinglePixel(currentData_->single_led_n, r, g, b, currentData_-> data[0].f);
        }
        else if (currentData_->control_type == currentData_->CTRL_TYPE_SPINNER)
        {
            uint8_t r = convertColor(currentData_->data[0].r);
            uint8_t g = convertColor(currentData_->data[0].g);
            uint8_t b = convertColor(currentData_->data[0].b);
            spinner(r, g, b, currentData_ -> animation_size, currentData_ -> animation_quantity, currentData_-> data[0].f);
        }
        else if (currentData_->control_type == currentData_->CTRL_TYPE_FAN)
        {
            uint8_t r = convertColor(currentData_->data[0].r);
            uint8_t g = convertColor(currentData_->data[0].g);
            uint8_t b = convertColor(currentData_->data[0].b);
            fan(r, g, b, true, currentData_ -> animation_quantity, currentData_-> data[0].f);
        }
        else
        {
            for (int i = 0; i < WS2812B_NUM_LEDS; i++)
            {
                uint8_t r = convertColor(currentData_->data[i].r);
                uint8_t g = convertColor(currentData_->data[i].g);
                uint8_t b = convertColor(currentData_->data[i].b);
                setSinglePixel(i, r, g, b, currentData_-> data[i].f);
            }
        }
        transmitSPI();
        frame++;
        }}
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
    void LEDNode::fillBuffer(uint8_t color, float intensity)
    {   
        uint8_t scaledColor = static_cast<uint8_t>(round(color * intensity));
        for (uint8_t mask = 0x80; mask; mask >>= 1)
        {
            if (scaledColor & mask)
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
    void LEDNode::setSinglePixel(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b, float frequency)
    {   auto mapping_pair = logicalToPhysicalMapping.find(pixel);
        const float phaseOffset = 3.14/4;  // Initial phase offset, if needed
        const float amplitudeOffset = 1.0f;  // Add 1 to sine wave to avoid negative values
        const float scaler = 2.0f;  // Scale the sine wave to 0-1
        const float intensity = (std::sin(frame / (2 * M_PI) * frequency + phaseOffset) + amplitudeOffset) / scaler;
        if (mapping_pair != logicalToPhysicalMapping.end())
        {
            uint16_t physicalID = mapping_pair->second;
            ptr = &ws2812b_buffer[24 * physicalID];
        }
        else
        {
            
            RCLCPP_INFO(this->get_logger(), "One of the logical LED IDs doesn't have a corresponding physical LED. "); 
            
        }
        
        fillBuffer(g,intensity);
        fillBuffer(r,intensity);
        fillBuffer(b,intensity);
    }

    void LEDNode::setAllPixels(uint8_t r, uint8_t g, uint8_t b, float frequency )
    {
        const float phaseOffset = 3.14/4;  // Initial phase offset, if needed
        const float amplitudeOffset = 1.0f;  // Add 1 to sine wave to avoid negative values
        const float scaler = 2.0f;  // Scale the sine wave to 0-1
        const float intensity = (std::sin(frame / (2 * M_PI) * frequency + phaseOffset) + amplitudeOffset) / scaler;
        ptr = ws2812b_buffer;
        
        for (uint16_t i = 0; i < WS2812B_NUM_LEDS; ++i)
        {
            fillBuffer(g,intensity);
            fillBuffer(r,intensity);
            fillBuffer(b,intensity);
        }
    }

    void LEDNode::spinner(uint8_t r, uint8_t g, uint8_t b,  uint8_t size, uint8_t blades, float frequency)
    {
        setAllPixels(0, 0, 0, 0);
        for (uint8_t i = 0; i < blades; i++)
        {
            for (uint32_t j = frame; j < frame + size; j++)
            {
                setSinglePixel((j + i * (WS2812B_NUM_LEDS / blades)) % WS2812B_NUM_LEDS,r, g, b, frequency);
            };
        };
        
        
    };

    void LEDNode::fan(uint8_t r, uint8_t g, uint8_t b,  bool opening, uint8_t blades, float frequency)
    {   uint8_t blade_length = WS2812B_NUM_LEDS / blades;
        uint8_t tip = frame % blade_length;
        setAllPixels(0, 0, 0, 0);
        for (uint32_t i = 0; i < blades; i++)
        {
            for (uint32_t j = 0; j < (opening ? tip : blade_length - tip); j++)
            {
                setSinglePixel((j + i * blade_length) % WS2812B_NUM_LEDS,r, g, b, frequency);
            };
        };

        
        
    };

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rae_hw::LEDNode);