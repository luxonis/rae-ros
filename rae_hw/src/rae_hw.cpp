#include "rae_hw/rae_hw.hpp"

#include <cmath>
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

    yaw_imu = 0.0;
    yaw_odom = 0.0;
    prev_yaw_imu = 0.0;
    prev_yaw_odom = 0.0;
    sumerr = 0.0;
    prev_err = 0.0;
    static_err = 0.0;
    static_correction = static_cast<bool>(std::stoi(info_.hardware_parameters["static_correction"]));
    ;
    PID pidIMU{std::stof(info_.hardware_parameters["PID_P_IMU"]),
               std::stof(info_.hardware_parameters["PID_I_IMU"]),
               std::stof(info_.hardware_parameters["PID_D_IMU"])};
    kp = pidIMU.P;
    ki = pidIMU.I;
    kd = pidIMU.D;
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=topic_based_ros2_control_"});
    node_ = rclcpp::Node::make_shared("_", options);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaeHW::on_configure(const rclcpp_lifecycle::State& /*previous state*/) {
    motorL->run();
    motorR->run();

    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>("/rae/imu/data", 10, std::bind(&RaeHW::imu_callback, this, std::placeholders::_1));
    odom_subscriber_ =
        node_->create_subscription<nav_msgs::msg::Odometry>("/diff_controller/odom", 10, std::bind(&RaeHW::odom_callback, this, std::placeholders::_1));

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
void RaeHW::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    float q_x = msg->orientation.x;
    float q_y = msg->orientation.y;
    float q_z = msg->orientation.z;
    float q_w = msg->orientation.w;

    // Calculate yaw from the quaternion
    yaw_imu = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (std::pow(q_y, 2) + std::pow(q_z, 2)));
}

void RaeHW::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double q_x = msg->pose.pose.orientation.x;
    double q_y = msg->pose.pose.orientation.y;
    double q_z = msg->pose.pose.orientation.z;
    double q_w = msg->pose.pose.orientation.w;

    yaw_odom = std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (std::pow(q_y, 2) + std::pow(q_z, 2)));
}

hardware_interface::return_type RaeHW::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if(rclcpp::ok())  // spin node
    {
        rclcpp::spin_some(node_);
    }
    float currLeftPos = motorL->getPos();
    float currRightPos = motorR->getPos();
    leftVel = motorL->getSpeed();
    rightVel = motorR->getSpeed();
    leftPos = currLeftPos;
    rightPos = currRightPos;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RaeHW::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
    double dt = period.seconds();
    auto adjustForWrap = [](double previousYaw, double currentYaw) {  // lambda for wraping, see nunmpy unwrap
        double delta = currentYaw - previousYaw;
        if(delta > M_PI) {
            currentYaw -= 2 * M_PI;
        } else if(delta < -M_PI) {
            currentYaw += 2 * M_PI;
        }
        return currentYaw;
    };

    double yaw_imu_wraped = adjustForWrap(prev_yaw_imu, yaw_imu);  // to handle -pi pi discontinuity
    double yaw_odom_wraped = adjustForWrap(prev_yaw_odom, yaw_odom - static_err);
    double err = yaw_odom_wraped - yaw_imu_wraped;
    double out = 0;

    if(static_correction) {
        sumerr += err * dt;
        if(sumerr > 10) {  // anti windup
            sumerr = 10;
        }
        if(sumerr < -10) {
            sumerr = -10;
        }
        float delta_err = (err - prev_err) / dt;
        if(yaw_imu) {  // prevent unwanted moves before initialization
            if(leftMotorCMD || rightMotorCMD) {
                out = err * kp + sumerr * ki + delta_err * kd;
            } else {
                out = (std::signbit(err) ? -3.95 : 3.95) + err * kp + sumerr * ki + delta_err * kd;
            }  // feedforward controller to bypass deadzone
        }
    } else {
        out = 0.0;
        if(abs(err) > 0.01) static_err += err;
    }
    leftMotorCMD -= out;
    rightMotorCMD += out;

    prev_yaw_imu = yaw_imu_wraped;
    prev_yaw_odom = yaw_odom_wraped;
    prev_err = err;

    motorL->motorSet(leftMotorCMD);
    motorR->motorSet(rightMotorCMD);
    return hardware_interface::return_type::OK;
}

}  // namespace rae_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rae_hw::RaeHW, hardware_interface::SystemInterface)
