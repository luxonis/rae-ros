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
#include <mutex>

namespace rae_hw
{

    struct State
    {
        int A;
        int B;
        bool operator==(const State &rhs)
        {
            return (A == rhs.A && B == rhs.B);
        }
        bool operator!=(const State &rhs)
        {
            return (A != rhs.A || B != rhs.B);
        }
    };

    class RaeMotor
    {
    private:
        volatile uint32_t dutyTarget = 0;
        volatile uint32_t dutyTrue = 0;
        std::atomic<bool> _running{true};
        gpiod::line pwmPin;
        gpiod::line phPin;
        gpiod::line enAPin;
        gpiod::line enBPin;
        float encRatio;
        bool direction = 0;
        bool encDirection;
        std::thread motorThread, encoderThread;
        void pwmMotor();
        void readEncoders();
        uint32_t speedToPWM(float speed);
        bool reversePhPinLogic_ = false;
        int prevCount;
        float rads;
        std::mutex encMtx;
        State Rest{0, 0};
        State Clockwise{0, 1};
        State Halfway{1, 1};
        State Counter{1, 0};
        State prevState;

    public:
        RaeMotor(const std::string &name, const std::string &chipName, int pwmPinNum, int phPinNum, int enA, int enB, float encTicsPerRev, bool reversePhPinLogic);
        ~RaeMotor();
        float getEncVal();
        void motorSet(float speed);

        void run();

        void stop();
    };

}
