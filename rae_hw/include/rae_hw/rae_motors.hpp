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
#include "gpiod.hpp"


namespace rae_hw{

class RaeMotor {
   private:
    volatile uint32_t dutyTarget = 0;
    volatile uint32_t dutyTrue = 0;
    std::atomic<bool> _running{true};
    int enPin;
    int phPin;
    bool direction = 0;
    std::thread motorThread, encoderThread;
    void pwmMotor() {
        while(_running) {
            if(dutyTrue) {
                write(enPin, "1", 1);
            }
            usleep(dutyTrue);
            write(enPin, "0", 1);
            usleep((1000 - dutyTrue));
            dutyTrue = dutyTarget;
        }
    }

   public:
    RaeMotor(int _enPin, int _phPin);

    void setSpeed(float _speed, bool _direction);

    void run() {
        _running = true;
        motorThread = std::thread(&RaeMotor::pwmMotor, this);
        if(direction) {
            write(phPin, "1", 1);
        } else {
            write(phPin, "0", 1);
        }
    }

    void stop() {
        _running = false;
        motorThread.join();
    }
};

class RaeRobotMotors {
   public:
    RaeMotor motorL;
    RaeMotor motorR;

   public:
    RaeRobotMotors(int pinEnL, int pinEnR, int pinPhL, int pinPhR) : motorL(pinEnL, pinPhL), motorR(pinEnR, pinPhR) {}

    void run() {
        motorL.run();
        motorR.run();
    }

};


}