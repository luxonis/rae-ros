#include "rae_hw/rae_hw.hpp"

#include <fstream>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rae_hw {

hardware_interface::CallbackReturn RaeHW::on_init(const hardware_interface::HardwareInfo& info) {
    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    leftWheelName = info_.hardware_parameters["left_wheel_name"];
    rightWheelName = info_.hardware_parameters["right_wheel_name"];
    auto chipName = info_.hardware_parameters["chip_name"];
    auto pwmName = info_.hardware_parameters["pwmName"];
    int pwmL = std::stoi(info_.hardware_parameters["pwmL"]);
    int phL = std::stoi(info_.hardware_parameters["phL"]);
    int enLA = std::stoi(info_.hardware_parameters["enLA"]);
    int enLB = std::stoi(info_.hardware_parameters["enLB"]);
    float encTicsPerRevL = std::stof(info_.hardware_parameters["encTicsPerRevL"]);
    float maxVelL = std::stof(info_.hardware_parameters["maxVelL"]);
    bool closedLoopL = static_cast<bool>(std::stoi(info_.hardware_parameters["closed_loopL"]));
    PID pidL{std::stof(info_.hardware_parameters["PID_P_L"]), std::stof(info_.hardware_parameters["PID_I_L"]), std::stof(info_.hardware_parameters["PID_D_L"])};
    motorL = std::make_unique<RaeMotor>(leftWheelName, chipName, pwmName, pwmL, phL, enLA, enLB, encTicsPerRevL, maxVelL, true, closedLoopL, pidL);

    int pwmR = std::stoi(info_.hardware_parameters["pwmR"]);
    int phR = std::stoi(info_.hardware_parameters["phR"]);
    int enRA = std::stoi(info_.hardware_parameters["enRA"]);
    int enRB = std::stoi(info_.hardware_parameters["enRB"]);
    float encTicsPerRevR = std::stof(info_.hardware_parameters["encTicsPerRevR"]);
    float maxVelR = std::stof(info_.hardware_parameters["maxVelR"]);
    bool closedLoopR = static_cast<bool>(std::stoi(info_.hardware_parameters["closed_loopR"]));
    PID pidR{std::stof(info_.hardware_parameters["PID_P_R"]), std::stof(info_.hardware_parameters["PID_I_R"]), std::stof(info_.hardware_parameters["PID_D_R"])};
    motorR = std::make_unique<RaeMotor>(rightWheelName, chipName, pwmName, pwmR, phR, enRA, enRB, encTicsPerRevR, maxVelR, false, closedLoopR, pidR);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaeHW::on_configure(const rclcpp_lifecycle::State& /*previous state*/) {
    motorL->run();
    motorR->run();

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RaeHW::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheelName, hardware_interface::HW_IF_POSITION, &leftPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheelName, hardware_interface::HW_IF_POSITION, &rightPos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(leftWheelName, hardware_interface::HW_IF_VELOCITY, &leftVel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(rightWheelName, hardware_interface::HW_IF_VELOCITY, &rightVel));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RaeHW::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(leftWheelName, hardware_interface::HW_IF_VELOCITY, &leftMotorCMD));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(rightWheelName, hardware_interface::HW_IF_VELOCITY, &rightMotorCMD));

    return command_interfaces;
}

hardware_interface::CallbackReturn RaeHW::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaeHW::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    motorL->stop();
    motorR->stop();

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaeHW::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    motorL->stop();
    motorR->stop();

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RaeHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    float currLeftPos = motorL->getPos();
    float currRightPos = motorR->getPos();
    leftVel = motorL->getSpeed();
    rightVel = motorR->getSpeed();
    leftPos = currLeftPos;
    rightPos = currRightPos;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RaeHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    motorL->motorSet(leftMotorCMD);
    motorR->motorSet(rightMotorCMD);

    return hardware_interface::return_type::OK;
}

}  // namespace rae_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rae_hw::RaeHW, hardware_interface::SystemInterface)
