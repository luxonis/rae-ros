#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            // Convert ROS image message to OpenCV image
            cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Open the framebuffer device
            int fbfd = open("/dev/fb0", O_RDWR);
            if (fbfd == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: cannot open framebuffer device");
                return;
            }

            // Get fixed screen information
            struct fb_fix_screeninfo finfo;
            if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading fixed information");
                return;
            }

            // Get variable screen information
            struct fb_var_screeninfo vinfo;
            if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading variable information");
                return;
            }

            // Calculate the size of the screen in bytes
            long screensize = vinfo.yres_virtual * finfo.line_length;

            // Map the device to memory
            char *fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, (off_t)0);
            if (fbp == MAP_FAILED)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: failed to map framebuffer device to memory");
                return;
            }

            // Resize to match the screen dimensions
            cv::resize(img, img, cv::Size(vinfo.xres, vinfo.yres));
            // Iterate over the image pixels
            for (int y = 0; y < img.rows; y++)
            {
                for (int x = 0; x < img.cols; x++)
                {
                    // Get pixel
                    cv::Vec3b pixel = img.at<cv::Vec3b>(cv::Point(x, y));

                    long location = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) +
                                    (y + vinfo.yoffset) * finfo.line_length;
                    *((cv::Vec3b *)(fbp + location)) = pixel;
                }
            }

            // Cleanup
            munmap(fbp, screensize);
            close(fbfd);
        };

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/battery_image", rclcpp::QoS(10), callback);
        RCLCPP_INFO(this->get_logger(), "NODE READY");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}