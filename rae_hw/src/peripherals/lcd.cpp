#include "rae_hw/peripherals/lcd.hpp"

#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace rae_hw {
LCDNode::LCDNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("lcd_node", options) {

}

LCDNode::~LCDNode() {
    cleanup();
}

void LCDNode::cleanup() {
    // Load default image
    cv::Mat default_img = cv::imread(default_logo_path);
    if(!default_img.empty()) {
        display_image(default_img);
    }

    munmap(fbp, screensize);
    close(fbfd);
}

CallbackReturn LCDNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    std::string logo_path = ament_index_cpp::get_package_share_directory("rae_hw") + "/assets/rae-logo-white.jpg";
    default_logo_path = declare_parameter<std::string>("logo_path", logo_path);

    // Open the framebuffer device
    fbfd = open("/dev/fb0", O_RDWR);
    if(fbfd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error: cannot open framebuffer device");
        return CallbackReturn::FAILURE;
    }

    // Get fixed screen information
    if(ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading fixed information");
        return CallbackReturn::FAILURE;
    }

    // Get variable screen information
    if(ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error reading variable information");
        return CallbackReturn::FAILURE;
    }

    // Calculate the size of the screen in bytes
    long screensize = vinfo.yres_virtual * finfo.line_length;

    // Map the device to memory
    fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, (off_t)0);
    if(fbp == MAP_FAILED) {
        RCLCPP_ERROR(this->get_logger(), "Error: failed to map framebuffer device to memory");
        return CallbackReturn::FAILURE;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("lcd", 10, std::bind(&LCDNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "LCD node configured!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LCDNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "LCD node activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LCDNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "LCD node deactivated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LCDNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    cleanup();
    RCLCPP_INFO(this->get_logger(), "LCD node shuttind down!");
    return CallbackReturn::SUCCESS;
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::LCDNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
