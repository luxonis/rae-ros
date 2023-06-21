#include "rae_hw/rae_motors.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_vel");
    auto pubL = node->create_publisher<geometry_msgs::msg::Twist>("left_vel",10);
    auto pubR = node->create_publisher<geometry_msgs::msg::Twist>("right_vel",10);
    float duration = 5.0;
    int encRatioL = 187;
    int encRatioR = 187;
    float speedL = 16.0;
    float speedR = 16.0;
    rae_hw::PID pid;

    if(argc == 2 && std::strcmp( argv[1], "-h" ) == 0 ){
        std::cout << "Help:\n";
        std::cout << "Positional arguments only for now.\n";
        std::cout << "ros2 run rae_hw test_encoders duration encRatioL encRatioR speedL speedR P I D.\n";
        std::cout << "With default arguments ros2 run rae_hw test_encoders 5.0 187 187 16.0 16.0 0.5 0.0 0.0" << std::endl;
        return 0;
    }
    if(argc > 1 && argc < 9){
        std::cout << "Please input all arguments in following form: \n";
        std::cout << "ros2 run rae_hw test_encoders duration encRatioL encRatioR speedL speedR P I D.\n";
        std::cout << "ros2 run rae_hw test_encoders 5.0 187 187 16.0 16.0 0.5 0.0 0.0" << std::endl;
    }
    else if(argc == 9){
        duration = atof(argv[1]);
        encRatioL = atoi(argv[2]);
        encRatioR = atoi(argv[3]);
        speedL = atof(argv[4]);
        speedR = atof(argv[5]);
        pid.P = atof(argv[6]);
        pid.I = atof(argv[7]);
        pid.D = atof(argv[8]);
    }
    std::cout << "Starting test procedure. Duration: " <<  duration  << "s.\n";
    std::cout << "Enc ratios - L: " << encRatioL << " R: " << encRatioR << " counts/rev." << std::endl;
    auto motorL = std::make_unique<rae_hw::RaeMotor>("left_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 2, 41, 46, 47, encRatioL, 32, true, true, pid);
    auto motorR = std::make_unique<rae_hw::RaeMotor>("right_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 1, 45, 42, 43, encRatioR, 32, false, true, pid);
    motorL->run();
    motorR->run();
    float leftPos = 0.0;
    float rightPos = 0.0;
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timePassed = false;
    while(!timePassed){
        geometry_msgs::msg::Twist leftVelMsg;
        geometry_msgs::msg::Twist rightVelMsg;
        motorL->motorSet(speedL);
        motorR->motorSet(speedR);
        leftVelMsg.linear.x = speedL;
        rightVelMsg.linear.x = speedR;
        auto currTime = std::chrono::high_resolution_clock::now();
        leftPos = motorL->getPos();
        rightPos = motorR->getPos();
        float currLeftVel = motorL->getSpeed();
        float currRightVel = motorR->getSpeed();

        leftVelMsg.linear.y = currLeftVel;
        leftVelMsg.linear.z = leftPos;
        rightVelMsg.linear.y = currRightVel;
        rightVelMsg.linear.z = rightPos;

        float startTimeDiff = std::chrono::duration<float>(currTime - startTime).count();
        if (startTimeDiff > duration){
            timePassed = true;
        }
        std::this_thread::sleep_for(10ms);
        pubL->publish(leftVelMsg);
        pubR->publish(rightVelMsg);
        rclcpp::spin_some(node);
    }
    startTime = std::chrono::high_resolution_clock::now();
    timePassed = false;

    rclcpp::shutdown();
    return 0;
}
