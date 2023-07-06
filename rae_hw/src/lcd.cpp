#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

class LCDNode : public rclcpp::Node {
public:
    LCDNode()
    : Node("image_subscriber") {
                    // Open the framebuffer device
            fbfd = open("/dev/fb0", O_RDWR);
            if (fbfd == -1) {
                RCLCPP_ERROR(this->get_logger(), "Error: cannot open framebuffer device");
                return;
            }

            // Get fixed screen information
            if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Error reading fixed information");
                return;
            }

            // Get variable screen information
            if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
                RCLCPP_ERROR(this->get_logger(), "Error reading variable information");
                return;
            }

            // Calculate the size of the screen in bytes
            long screensize = vinfo.yres_virtual * finfo.line_length;

            // Map the device to memory
            fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, (off_t)0);
            if (fbp == MAP_FAILED) {
                RCLCPP_ERROR(this->get_logger(), "Error: failed to map framebuffer device to memory");
                return;
            }

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "lcd", 10, std::bind(&LCDNode::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "LCD node running!");
    }
    ~LCDNode(){
            munmap(fbp, screensize);
            close(fbfd);
    }


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            // Convert ROS image message to OpenCV image
            cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
            // Resize to match the screen dimensions
            cv::resize(img, img, cv::Size(vinfo.xres, vinfo.yres));
            // Iterate over the image pixels
            for (int y = 0; y < img.rows; y++) {
                for (int x = 0; x < img.cols; x++) {
                    // Get pixel
                    cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));

                    long location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                                    (y+vinfo.yoffset) * finfo.line_length;
                    *((cv::Vec3b*)(fbp + location)) = pixel;
                }
            }
        };
    

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int fbfd;
    char *fbp;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long screensize;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCDNode>());
    rclcpp::shutdown();
    return 0;
}