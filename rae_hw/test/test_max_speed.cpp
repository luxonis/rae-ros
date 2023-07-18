#include "rae_hw/rae_motors.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <cstring>

using namespace std::chrono_literals;

int main(int argc, char *argv[]){
    float duration = 5.0;
    int encRatioL = 756;
    int encRatioR = 756;

    if(argc == 2 && std::strcmp( argv[1], "-h" ) == 0 ){
        std::cout << "Help:\n";
        std::cout << "Positional arguments only for now.\n";
        std::cout << "ros2 run rae_hw test_encoders duration encRatioL encRatioR.\n";
        std::cout << "With default arguments ros2 run rae_hw test_encoders 5.0 187 187" << std::endl;
        return 0;
    }
    if(argc > 1 && argc < 4){
        std::cout << "Please input all arguments in following form: \n";
        std::cout << "ros2 run rae_hw test_encoders duration encRatioL encRatioR.\n";
        std::cout << "ros2 run rae_hw test_encoders 5.0 187 187" << std::endl;
    }
    else if(argc == 4){
        duration = atof(argv[1]);
        encRatioL = atoi(argv[2]);
        encRatioR = atoi(argv[3]);
    }
    std::cout << "Starting test procedure. Duration: " <<  duration  << "s.\n";
    std::cout << "Enc ratios - L: " << encRatioL << " R: " << encRatioR << " counts/rev." << std::endl;
    auto motorR = std::make_unique<rae_hw::RaeMotor>("right_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 2, 45, 46, 47, encRatioR, 32, false);
    auto motorL = std::make_unique<rae_hw::RaeMotor>("left_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 1, 41, 42, 43, encRatioL, 32, true);
    motorL->run();
    motorR->run();
    float prevMaxVelL = 0.0;
    float prevMaxVelR = 0.0;
    float leftPos = 0.0;
    float rightPos = 0.0;
    auto startTime = std::chrono::high_resolution_clock::now();
    auto prevTime = startTime;
    bool timePassed = false;
    while(!timePassed){
        motorL->setPWM(150000, 100000);
        motorR->setPWM(150000, 100000);
        auto currTime = std::chrono::high_resolution_clock::now();
        float currLeftPos = motorL->getPos();
        float currRightPos = motorR->getPos();
        float timeDiff = std::chrono::duration<float>(currTime - prevTime).count();
        std::cout << "Test finished. \n Max speed L: " << currRightPos << " rads/s.\n Max speed R: " << currLeftPos << " rads/s." << std::endl;
        float leftVel = (currLeftPos - leftPos) / timeDiff;
        float rightVel = (currRightPos - rightPos) / timeDiff;
        prevTime = currTime;
        leftPos = currLeftPos;
        rightPos = currRightPos;

        if(leftVel > prevMaxVelL){
            prevMaxVelL = leftVel;
        }
        if(rightVel > prevMaxVelR){
            prevMaxVelR = rightVel;
        }
        float startTimeDiff = std::chrono::duration<float>(currTime - startTime).count();
        if (startTimeDiff > duration){
            timePassed = true;
        }
        std::this_thread::sleep_for(20ms);
    }
    motorL->disablePWM();
    motorR->disablePWM();

    std::cout << "Test finished. \n Max speed L: " << prevMaxVelL << " rads/s.\n Max speed R: " << prevMaxVelR << " rads/s." << std::endl;

    return 0;
}
