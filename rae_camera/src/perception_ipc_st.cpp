#include "rclcpp/rclcpp.hpp"
#include "depthai_ros_driver/camera.hpp"
#include "laserscan_kinect/laserscan_kinect_node.hpp"
#include "ira_laser_tools/laserscan_multi_merger.hpp"
#include "slam_toolbox/slam_toolbox_sync.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto camera_opts = opts;
  auto camera = std::make_unique<depthai_ros_driver::Camera>(camera_opts);

  auto camera_name = camera->get_name();

  auto scan_front_opts = opts;
  scan_front_opts.arguments({"--ros-args", "--remap", "laserscan_kinect:__node:=laserscan_kinect_front",
                             "--remap", std::string("/image:=") + camera_name + std::string("/stereo_front/image_raw"),
                             "--remap", std::string("/camera_info:=") + camera_name + std::string("/stereo_front/camera_info"),
                             "--remap", std::string("/scan:=") + camera_name + std::string("/scan_front"),
                             "--remap", std::string("/debug_image:=") + camera_name + std::string("/debug_image_front"),
                             "--remap", std::string("/debug_image/compressed:=") + camera_name + std::string("/debug_image_front/compressed")
                             });
  auto laserscanFront = std::make_unique<laserscan_kinect::LaserScanKinectNode>(scan_front_opts);
  auto scan_back_opts = opts;
  scan_back_opts.arguments({"--ros-args", "--remap", "laserscan_kinect:__node:=laserscan_kinect_back",
                             "--remap", std::string("/image:=") + camera_name + std::string("/stereo_back/image_raw"),
                             "--remap", std::string("/camera_info:=") + camera_name + std::string("/stereo_back/camera_info"),
                             "--remap", std::string("/scan:=") + camera_name + std::string("/scan_back"),
                             "--remap", std::string("/debug_image:=") + camera_name + std::string("/debug_image_back"),
                             "--remap", std::string("/debug_image/compressed:=") + camera_name + std::string("/debug_image_back/compressed")
                             });
  auto laserscanBack = std::make_unique<laserscan_kinect::LaserScanKinectNode>(scan_back_opts);

  auto merger = std::make_unique<ira_laser_tools::LaserscanMerger>(opts);
  std::shared_ptr<slam_toolbox::SynchronousSlamToolbox> slam_toolbox;

    int stack_size = 40000000;
    {
      auto temp_node = std::make_shared<rclcpp::Node>("slam_toolbox");
      temp_node->declare_parameter("stack_size_to_use",rclcpp::ParameterType::PARAMETER_INTEGER);
      if (temp_node->get_parameter("stack_size_to_use", stack_size)) {
        RCLCPP_INFO(temp_node->get_logger(), "Node using stack size %i", (int)stack_size);
        const rlim_t max_stack_size = stack_size;
        struct rlimit stack_limit;
        getrlimit(RLIMIT_STACK, &stack_limit);
        if (stack_limit.rlim_cur < stack_size) {
          stack_limit.rlim_cur = stack_size;
        }
        setrlimit(RLIMIT_STACK, &stack_limit);
      }
    }
    slam_toolbox = std::make_shared<slam_toolbox::SynchronousSlamToolbox>(rclcpp::NodeOptions());
    slam_toolbox->configure();
    slam_toolbox->loadPoseGraphByParams();
  

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(camera->get_node_base_interface());
  executor.add_node(laserscanFront->get_node_base_interface());
  executor.add_node(laserscanBack->get_node_base_interface());
  executor.add_node(merger->get_node_base_interface());
  executor.add_node(slam_toolbox->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}