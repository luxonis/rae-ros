#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rae_hw {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class BatteryNode : public rclcpp_lifecycle::LifecycleNode {
   public:
    BatteryNode(const rclcpp::NodeOptions& options);
    ~BatteryNode();

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

   private:
    std::string readVarFromFile(const std::string& varName);
    void logStatus(const sensor_msgs::msg::BatteryState& message);
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
    sensor_msgs::msg::BatteryState prevState;
    rclcpp::Time stateChangeTime;
    rclcpp::Time lastLogTime;
    std::unordered_map<std::string, uint8_t> stringToStateMsg{{"Charging", sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING},
                                                              {"Discharging", sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING}};
    std::unordered_map<uint8_t, std::string> stateMsgToString{
        {sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING, "Charging"},
        {sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, "Discharging"},
    };
};
}  // namespace rae_hw
