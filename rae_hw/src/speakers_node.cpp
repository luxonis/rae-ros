#include "rclcpp/rclcpp.hpp"
#include "rae_hw/peripherals/speakers.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::SpeakersNode>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}