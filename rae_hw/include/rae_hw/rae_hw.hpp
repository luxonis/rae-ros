#ifndef RAE_HW__RAE_HW_HPP_
#define RAE_HW__RAE_HW_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "gpiod.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rae_hw/rae_motors.hpp"
#include "rae_hw/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rae_hw {

class RaeHW : public hardware_interface::SystemInterface {
   public:
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   private:
    std::unique_ptr<RaeMotor> motorL, motorR;
    double leftMotorCMD, rightMotorCMD;
    int pwmA, pwmB, phA, phB;
    double leftPos, rightPos, leftVel, rightVel;
    std::string leftWheelName, rightWheelName;
};

}  // namespace rae_hw

#endif  // RAE_HW__RAE_HW_HPP_