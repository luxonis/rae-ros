#include <math.h>
#include "rae_hw/rae_motors.hpp"

namespace rae_hw

{
    using namespace std::chrono_literals;
    RaeMotor::RaeMotor(const std::string &name,
                       const std::string &chipName,
                       int pwmPinNum,
                       int phPinNum,
                       int enA,
                       int enB,
                       float encTicsPerRev,
                       float maxVel,
                       bool reversePhPinLogic,
                       bool closedLoop,
                       PID pid)
    {
        gpiod::chip chip(chipName);
        pwmPin = chip.get_line(pwmPinNum);
        phPin = chip.get_line(phPinNum);
        enAPin = chip.get_line(enA);
        enBPin = chip.get_line(enB);
        pwmPin.request({name + "_pwm", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
        phPin.request({name + "_ph", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
        enAPin.request({name + "_en_a", gpiod::line_request::DIRECTION_INPUT, 0});
        enBPin.request({name + "_en_b", gpiod::line_request::DIRECTION_INPUT, 0});
        int A = enAPin.get_value();
        int B = enBPin.get_value();
        prevState = State{A, B};
        encDirection = true;
        reversePhPinLogic_ = reversePhPinLogic;
        encRatio = 2.0 * M_PI / encTicsPerRev;
        velLim = maxVel;
        closedLoop_ = closedLoop;
        prevCount = 0;
        rads = 0.0;
        currPID = pid;
        prevError = 0.0;
        errSum = 0.0;
        prevPos = 0.0;
        prevVelTime = std::chrono::high_resolution_clock::now();
        prevErrorTime = std::chrono::high_resolution_clock::now();
    }
    RaeMotor::~RaeMotor()
    {
        stop();
    }

    void RaeMotor::calcSpeed()
    {
        while (_running)
        {
            auto currTime = std::chrono::high_resolution_clock::now();
            float currPos = getPos();
            float timeDiff = std::chrono::duration<float>(currTime - prevVelTime).count();
            {
                std::lock_guard<std::mutex> lck(speedMtx);
                currentSpeed = (currPos - prevPos) / timeDiff;
            }
            prevVelTime = currTime;
            prevPos = currPos;
            std::this_thread::sleep_for(15ms);
        }
    }

    void RaeMotor::controlSpeed()
    {
        while (_running)
        {
            uint32_t speedMil;
            bool dir;
            if (closedLoop_)
            {
                auto currTime = std::chrono::high_resolution_clock::now();
                float timeDiff = std::chrono::duration<float>(currTime - prevErrorTime).count();

                float currSpeed = getSpeed();
                float error = targetSpeed - currSpeed;
                float eP = error * currPID.P;
                errSum += (error * timeDiff);
                float eI = errSum * currPID.I;
                float eD = (error - prevError) / timeDiff * currPID.D;
                float outSpeed = targetSpeed + eP + eI + eD;
                dir = (outSpeed >= 0) ^ reversePhPinLogic_;
                speedMil = speedToPWM(outSpeed);
                prevErrorTime = currTime;
                prevError = error;
            }
            else
            {
                speedMil = speedToPWM(targetSpeed);
                dir = (targetSpeed >= 0) ^ reversePhPinLogic_;
            }
            if (dir == motDirection)
            {
                dutyTarget = speedMil;
            }
            else
            {
                // Switch direction
                // First stop
                dutyTarget = 0;
                while (dutyTrue != 0)
                {
                    usleep(100);
                }
                motDirection = dir;
                if (motDirection)
                {
                    phPin.set_value(1);
                }
                else
                {
                    phPin.set_value(0);
                }

                // Set speed
                dutyTarget = speedMil;
            }
            std::this_thread::sleep_for(5ms);
        }
    }
    void RaeMotor::pwmMotor()
    {
        while (_running)
        {
            if (dutyTrue)
            {
                pwmPin.set_value(1);
            }
            usleep(dutyTrue);
            pwmPin.set_value(0);
            usleep((1000 - dutyTrue));
            dutyTrue = dutyTarget;
        }
    }
    void RaeMotor::readEncoders()
    {
        while (_running)
        {
            usleep(1);
            int currA = enAPin.get_value();
            int currB = enBPin.get_value();
            State currS{currA, currB};
            int count = prevCount;
            if (currS == Clockwise)
            {
                if (prevState == Rest)
                {
                    encDirection = true;
                }
                else
                {
                    encDirection = false;
                }
            }
            else if (currS == Counter)
            {
                if (prevState == Rest)
                {
                    encDirection = false;
                }
                else
                {
                    encDirection = true;
                }
            }
            else if (currS == Rest && currS != prevState)
            {
                if (encDirection)
                {
                    count++;
                }
                else
                {
                    count--;
                }
            }
            if (count != prevCount)
            {
                {
                    std::lock_guard<std::mutex> lck(posMtx);
                    rads = count * encRatio;
                }
                prevCount = count;
            }
            prevState = currS;
        }
    }
    float RaeMotor::getPos()
    {
        std::lock_guard<std::mutex> lck(posMtx);
        return rads;
    }
    float RaeMotor::getSpeed()
    {
        std::lock_guard<std::mutex> lck(speedMtx);
        return currentSpeed;
    }

    uint32_t RaeMotor::speedToPWM(float speed)
    {
        float normSpeed = speed / velLim * 100;
        float clSpeedNorm = std::clamp(normSpeed, -100.0f, 100.0f);
        uint32_t normduty = static_cast<uint32_t>(std::abs(clSpeedNorm) * 10.0);
        return normduty;
    }
    void RaeMotor::motorSet(float speed)
    {
        targetSpeed = speed;
    }

    void RaeMotor::run()
    {
        _running = true;
        motorThread = std::thread(&RaeMotor::pwmMotor, this);
        encoderThread = std::thread(&RaeMotor::readEncoders, this);
        calcSpeedThread = std::thread(&RaeMotor::calcSpeed, this);
        speedControlThread = std::thread(&RaeMotor::controlSpeed, this);
        if (motDirection)
        {
            phPin.set_value(1);
        }
        else
        {
            phPin.set_value(0);
        }
    }

    void RaeMotor::stop()
    {
        _running = false;
        motorThread.join();
        encoderThread.join();
        calcSpeedThread.join();
        speedControlThread.join();
        pwmPin.set_value(0);
        pwmPin.release();
        phPin.set_value(0);
        phPin.release();
        enAPin.release();
        enBPin.release();
    }

}