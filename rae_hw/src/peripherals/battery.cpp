#include "rae_hw/peripherals/battery.hpp"

#include <unistd.h>

#include <fstream>
#include <iostream>

namespace rae_hw {
BatteryNode::BatteryNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("battery_node", options) {

}
BatteryNode::~BatteryNode() = default;

CallbackReturn BatteryNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    using namespace std::chrono_literals;
    publisher = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_status", 10);
    timer = this->create_wall_timer(500ms, std::bind(&BatteryNode::timerCallback, this));
    stateChangeTime = this->get_clock()->now();
    lastLogTime = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Battery node configured!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn BatteryNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Battery node activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn BatteryNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Battery node deactivated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn BatteryNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Battery node shuttind down!");
    return CallbackReturn::SUCCESS;
}

void BatteryNode::timerCallback() {
    auto message = sensor_msgs::msg::BatteryState();
    message.header.stamp = this->get_clock()->now();
    try {
        message.capacity = std::stof(readVarFromFile("capacity"));
        message.power_supply_status = stringToStateMsg.at(readVarFromFile("status"));
    } catch(std::invalid_argument const& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
    logStatus(message);
    publisher->publish(message);
    prevState = message;
}

std::string BatteryNode::readVarFromFile(const std::string& varName) {
    std::string fileName = "/sys/class/power_supply/bq27441-0/" + varName;
    std::ifstream ifstrm(fileName, std::ios::in);
    std::string s;
    if(ifstrm.is_open()) {
        ifstrm >> s;
        ifstrm.close();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read from %s", fileName.c_str());
    }
    return s;
}

void BatteryNode::logStatus(const sensor_msgs::msg::BatteryState& message) {
    auto currTime = this->get_clock()->now();
    auto stateTimeDiff = currTime - stateChangeTime;
    int stateSecs = (int)stateTimeDiff.seconds();
    int stateMins = stateSecs / 60;
    int stateHours = stateMins / 60;

    if(message.power_supply_status != prevState.power_supply_status) {
        RCLCPP_INFO(this->get_logger(),
                    "Power supply status changed to [%s] after %d h %d min %d secs.",
                    stateMsgToString.at(message.power_supply_status).c_str(),
                    int(stateHours),
                    int(stateMins % 60),
                    int(stateSecs % 60));
        stateChangeTime = currTime;
        stateSecs = stateMins = stateHours = 0;
    }
    if(message.capacity < 100.0 && message.capacity != prevState.capacity && (int)message.capacity % 10 == 0) {
        auto logTimeDiff = currTime - lastLogTime;
        int logSecs = (int)logTimeDiff.seconds();
        int logMins = logSecs / 60;
        int logHours = logMins / 60;

        RCLCPP_INFO(this->get_logger(),
                    "Battery capacity: %f, Status: [%s] for %d h %d min %d s. Time since last log: %d h %d min "
                    "%d secs.",
                    message.capacity,
                    stateMsgToString.at(message.power_supply_status).c_str(),
                    int(stateHours),
                    int(stateMins % 60),
                    int(stateSecs % 60),
                    int(logHours),
                    int(logMins % 60),
                    int(logSecs % 60));
        lastLogTime = currTime;
    } else if(message.capacity < 30.0 && message.capacity != prevState.capacity && (int)message.capacity % 5 == 0) {
        RCLCPP_WARN(this->get_logger(), "Battery status low! Current capacity: %f", message.capacity);
    }
}
}  // namespace rae_hw
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::BatteryNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
