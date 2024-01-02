#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <cstring>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rae_hw
{
class LCDNode : public rclcpp::Node
{
public:
  LCDNode(const rclcpp::NodeOptions & options);
  ~LCDNode();

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void display_image(const cv::Mat & img);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  int fbfd;
  char * fbp;
  struct fb_var_screeninfo vinfo;
  struct fb_fix_screeninfo finfo;
  long screensize;
  std::string default_logo_path;
};
}  // namespace rae_hw