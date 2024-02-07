#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <cstring>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rae_hw {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class LCDNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    LCDNode(const rclcpp::NodeOptions& options);
    ~LCDNode();

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void display_image(const cv::Mat& img);
    void cleanup();

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int fbfd;
    char* fbp;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long screensize;
    std::string default_logo_path;
};
}  // namespace rae_hw