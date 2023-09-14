#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
namespace rae_hw
{
    class LCDNode : public rclcpp::Node
    {
    public:
        LCDNode(const rclcpp::NodeOptions &options);
        ~LCDNode();

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        int fbfd;
        char *fbp;
        struct fb_var_screeninfo vinfo;
        struct fb_fix_screeninfo finfo;
        long screensize;
    };
}