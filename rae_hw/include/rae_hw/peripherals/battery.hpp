#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rae_hw
{
class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode(const rclcpp::NodeOptions & options);
  ~BatteryNode();

private:
  std::string readVarFromFile(const std::string & varName);
  void logStatus(const sensor_msgs::msg::BatteryState & message);
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
  sensor_msgs::msg::BatteryState prevState;
  rclcpp::Time stateChangeTime;
  rclcpp::Time lastLogTime;
  std::unordered_map<std::string, uint8_t> stringToStateMsg{
    {"Charging", sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING},
    {"Discharging", sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING}};
  std::unordered_map<uint8_t, std::string> stateMsgToString{
    {sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING, "Charging"},
    {sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING, "Discharging"},
  };
};
}  // namespace rae_hw
