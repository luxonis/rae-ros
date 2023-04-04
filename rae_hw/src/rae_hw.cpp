// Copyright (c) 2023, Luxonis
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "rae_hw/rae_hw.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rae_hw
{
  hardware_interface::CallbackReturn RaeHW::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    // TODO(anyone): read parameters and initialize the hardware

    prevTime = std::chrono::high_resolution_clock::now();
    leftWheelName = info_.hardware_parameters["left_wheel_name"];
    rightWheelName = info_.hardware_parameters["right_wheel_name"];
    auto chipName = info_.hardware_parameters["chip_name"];

    int pwmL = std::stoi(info_.hardware_parameters["pwmL"]);
    int phL = std::stoi(info_.hardware_parameters["phL"]);
    int enLA = std::stoi(info_.hardware_parameters["enLA"]);
    int enLB = std::stoi(info_.hardware_parameters["enLB"]);
    float encTicsPerRevL = std::stof(info_.hardware_parameters["encTicsPerRevL"]);
    float maxVelL = std::stof(info_.hardware_parameters["maxVelL"]);

    motorL = std::make_unique<RaeMotor>(leftWheelName,
                                        chipName, pwmL, phL, enLA, enLB, encTicsPerRevL, maxVelL, true);

    int pwmR = std::stoi(info_.hardware_parameters["pwmR"]);
    int phR = std::stoi(info_.hardware_parameters["phR"]);
    int enRA = std::stoi(info_.hardware_parameters["enRA"]);
    int enRB = std::stoi(info_.hardware_parameters["enRB"]);
    float encTicsPerRevR = std::stof(info_.hardware_parameters["encTicsPerRevR"]);
    float maxVelR = std::stof(info_.hardware_parameters["maxVelR"]);

    motorR = std::make_unique<RaeMotor>(rightWheelName,
                                        chipName, pwmR, phR, enRA, enRB, encTicsPerRevR, maxVelR, false);
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RaeHW::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    motorL->run();
    motorR->run();
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RaeHW::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        leftWheelName, hardware_interface::HW_IF_POSITION, &leftPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rightWheelName, hardware_interface::HW_IF_POSITION, &rightPos));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        leftWheelName, hardware_interface::HW_IF_VELOCITY, &leftVel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        rightWheelName, hardware_interface::HW_IF_VELOCITY, &rightVel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RaeHW::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        leftWheelName, hardware_interface::HW_IF_VELOCITY, &leftMotorCMD));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        rightWheelName, hardware_interface::HW_IF_VELOCITY, &rightMotorCMD));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn RaeHW::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // TODO(anyone): prepare the robot to receive commands
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RaeHW::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    motorL->stop();
    motorR->stop();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn RaeHW::on_shutdown(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    motorL->stop();
    motorR->stop();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RaeHW::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto currTime = std::chrono::high_resolution_clock::now();
    float currLeftPos = motorL->getPos();
    float currRightPos = motorR->getPos();
    float timeDiff = std::chrono::duration<float>(currTime - prevTime).count();
    leftVel = (currLeftPos - leftPos) / timeDiff;
    rightVel = (currRightPos - rightPos) / timeDiff;
    prevTime = currTime;
    leftPos = currLeftPos;
    rightPos = currRightPos;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RaeHW::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    motorL->motorSet(leftMotorCMD);
    motorR->motorSet(rightMotorCMD);
    return hardware_interface::return_type::OK;
  }

} // namespace rae_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rae_hw::RaeHW, hardware_interface::SystemInterface)
