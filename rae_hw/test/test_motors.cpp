#include "rae_hw/rae_motors.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <cstring>


int main(int argc, char *argv[]){
    float duration = 5.0;
    float speedL = 16.0;
    float speedR = 16.0;
    int encRatioL = 756;
    int encRatioR = 756;
    int maxVelL = 32;
    int maxVelR = 32;

    if(argc == 2 && std::strcmp( argv[1], "-h" ) == 0 ){
        std::cout << "Help:\n";
        std::cout << "Positional arguments only for now.\n";
        std::cout << "ros2 run rae_hw test_motors duration speedL speedR encRatioL encRatioR maxVelL maxVelR.\n";
        std::cout << "With default arguments ros2 run rae_hw test_motors 5.0 16.0 16.0 187 187 32 32" << std::endl;
        return 0;
    }
    if(argc > 1 && argc < 8){
        std::cout << "Please input all arguments in following form: \n";
        std::cout << "ros2 run rae_hw test_motors duration speedL speedR encRatioL encRatioR maxVelL maxVelR.\n";
        std::cout << "ros2 run rae_hw test_motors 5.0 16.0 16.0 187 187 32 32" << std::endl;
    }
    else if(argc == 8){
        duration = atof(argv[1]);
        speedL = atof(argv[2]);
        speedR = atof(argv[3]);
        encRatioL = atoi(argv[4]);
        encRatioR = atoi(argv[5]);
        maxVelL = atoi(argv[6]);
        maxVelR = atoi(argv[7]);
    }
    std::cout << "Starting test procedure. Duration: " <<  duration  << "s.\n";
    std::cout << "Motor speeds - L: " << speedL << " R: " << speedR << " rad/s.\n";
    std::cout << "Enc ratios - L: " << encRatioL << " R: " << encRatioR << " counts/rev.\n";
    std::cout << "Max speeds - L: " << maxVelL << " R: " << maxVelR << " rads/s." << std::endl;
    auto motorL = std::make_unique<rae_hw::RaeMotor>("left_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 1, 41, 42, 43, encRatioL, maxVelL, true);
    auto motorR = std::make_unique<rae_hw::RaeMotor>("right_wheel_name",
                                        "gpiochip0", "/sys/class/pwm/pwmchip0", 2, 45, 46, 47, encRatioR, maxVelR, false);
    
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timePassed = false;
    while(!timePassed){
        
        motorL->setPWM(150000, 100000);
        motorR->setPWM(150000, 100000);
        auto currTime = std::chrono::high_resolution_clock::now();
        float timeDiff = std::chrono::duration<float>(currTime - startTime).count();
        if (timeDiff > duration){
            timePassed = true;
        }
    }
    auto leftPos = motorL->getPos();
    auto rightPos = motorR->getPos();
    motorL->disablePWM();
    motorR->disablePWM();
    std::cout << "Test complete after " << duration << "s.\n";
    std::cout << "Left pos: " << leftPos << " rad.\n";
    std::cout <<  "Right pos: " << rightPos << " rad." << std::endl;
        

    return 0;
}

