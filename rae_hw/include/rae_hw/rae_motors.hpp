#pragma once

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <gpiod.hpp>



namespace rae_hw{

class RaeMotor
{
private:
    volatile uint32_t dutyTarget = 0;
    volatile uint32_t dutyTrue = 0;
    std::atomic<bool> _running{true};
    int pwmPin;
    int phPin;
    bool direction = 0;
    std::thread motorThread, encoderThread;
    void pwmMotor();
    void readEncoders();
    uint32_t speedToPWM(float speed);
    bool reversePhPinLogic_ = false;
   public:
    RaeMotor(const std::string& name, const std::string& chipName, int pwmPinNum, int phPinNum, bool reversePhPinLogic);
    ~RaeMotor();
    void motorSet(float speed);

    void run();

    void stop();
    
};


}
