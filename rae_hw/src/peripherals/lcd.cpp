#include "rae_hw/peripherals/lcd.hpp"

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rae_hw {
LCDNode::LCDNode(const rclcpp::NodeOptions& options) : Node("lcd_node", options) {
    std::string logo_key = "default_logo_path";
    declare_parameter<std::string>(logo_key);
    default_logo_path = get_parameter(logo_key).as_string();

    // Open the framebuffer device
    fbfd = open("/dev/fb0", O_RDWR);
    if(fbfd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error: cannot open framebuffer device");
        return;
    }

    // Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading fixed information");
        return;
    }

    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading variable information");
        return;
    }

    // Calculate the size of the screen in bytes
    long screensize = vinfo.yres_virtual * finfo.line_length;

    // Map the device to memory
    fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, (off_t)0);
    if(fbp == MAP_FAILED) {
        RCLCPP_ERROR(this->get_logger(), "Error: failed to map framebuffer device to memory");
        return;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("lcd", 10, std::bind(&LCDNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "LCD node running!");
}

LCDNode::~LCDNode() {
    // Load default image
    cv::Mat default_img = cv::imread(default_logo_path);
    if(!default_img.empty()) {
        display_image(default_img);
    }

    munmap(fbp, screensize);
    close(fbfd);
}

void LCDNode::display_image(const cv::Mat& img) {
    // Resize to match the screen dimensions
    cv::resize(img, img, cv::Size(vinfo.xres, vinfo.yres));
    // Iterate over the image pixels
    for(int y = 0; y < img.rows; y++) {
        for(int x = 0; x < img.cols; x++) {
            // Get pixel
            cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));

            long location = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) + (y + vinfo.yoffset) * finfo.line_length;
            *((cv::Vec3b*)(fbp + location)) = pixel;
        }
    }
}

void LCDNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image message to OpenCV image
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    display_image(img);
};
}  // namespace rae_hw

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rae_hw::LCDNode);
