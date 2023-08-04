#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <fstream>
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
    /// @brief Struct representing encoder GPIO states. Following states are used to calculate rotation:
    ///
    ///
    /// Rest{0, 0} Clockwise{0, 1} Halfway{1, 1} Counter{1, 0}
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
    struct PID
    {
        float P;
        float I;
        float D;
    };
    /// @brief Class responsible for motor control and encoder readout.
    /// PWM generation and encoder readout are happening in separate threads.
    class RaeMotor
    {
    public:
        RaeMotor(const std::string &name, 
                 const std::string &chipName, 
                 const std::string &pwmName,
                 int pwmPinNum, 
                 int phPinNum, 
                 int enA, 
                 int enB, 
                 float encTicsPerRev, 
                 float maxVel, 
                 bool reversePhPinLogic, 
                 bool closedLoop = false, 
                 PID pid = {0.6,0.4,0.0});
        ~RaeMotor();
        /// @brief Get current motor position.
        /// @return Position in rads.
        float getPos();
        float getSpeed();
        /// @brief Set motor speed.
        /// @param speed rads/s.
        void motorSet(float speed);
        /// @brief Starts threads for PWM generation and encoder readout.
        void run();
        /// @brief Stops motors, joins threads and frees GPIO pins.
        void stop();
        void setPWM(int period, int duty_cycle);
        void disablePWM();

    private:
        /// @brief Thread function generating PWM based on dutyTarget.
        void pwmMotor();
        /// @brief Thread function reading encoder GPIO values and calculating current motor position.
        /// Encoder count is calculated based on a small state machine.
        /// Clockwise direction: 00 -> 01 -> 11 -> 10 -> 00
        /// Counterclockwise: 00 -> 10 -> 11 -> 01 -> 00
        /// Position in rads is calculated based on given encoder ratio (ticks/rev).
        void readEncoders();
        /// @brief Converts speed to PWM cycle. Conversion is based on set max speed.
        /// @param speed in rads/s
        /// @return PWM target in range [0:1000]us
        uint32_t speedToPWM(float speed);
        float calcSpeed();
        void controlSpeed();
        volatile uint32_t dutyTarget = 0;
        volatile uint32_t dutyTrue = 0;
        float targetSpeed;
        float currentSpeed;
        float prevPos;
        float prevError;
        float errSum;
        std::atomic<bool> _running{true};
        int pwmPin;
        gpiod::line phPin;
        gpiod::line enAPin;
        gpiod::line enBPin;
        float encRatio;
        float velLim;
        bool motDirection = 0;
        bool encDirection;
        std::thread motorThread, encoderThread, calcSpeedThread, speedControlThread;
        bool reversePhPinLogic_ = false;
        std::string pwmName_="/sys/class/pwm/pwmchip0";
        bool closedLoop_ = true;
        int prevCount;
        float rads;
        std::mutex posMtx, speedMtx;
        const State Rest{0, 0};
        const State Clockwise{0, 1};
        const State Halfway{1, 1};
        const State Counter{1, 0};
        State prevState;
        PID currPID;
        
        std::chrono::high_resolution_clock::time_point prevVelTime;
        std::chrono::high_resolution_clock::time_point prevErrorTime;
    };

}
