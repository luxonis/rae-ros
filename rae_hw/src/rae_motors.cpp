#include <math.h>
#include "rae_hw/rae_motors.hpp"

namespace rae_hw

{
    RaeMotor::RaeMotor(const std::string &name, const std::string &chipName, int pwmPinNum, int phPinNum, int enA, int enB, float encTicsPerRev, bool reversePhPinLogic)
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
        prevCount = 0;
    }
    RaeMotor::~RaeMotor()
    {
        stop();
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
            usleep((100));
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
                std::lock_guard<std::mutex> lck(encMtx);
                rads = count * encRatio;
                prevCount = count;
            }
            prevState = currS;
        }
    }
    float RaeMotor::getEncVal()
    {
        if (reversePhPinLogic_)
        {
            std::lock_guard<std::mutex> lck(encMtx);
            return -rads;
        }
        else
        {
            std::lock_guard<std::mutex> lck(encMtx);
            return rads;
        }
    }

    uint32_t RaeMotor::speedToPWM(float speed)
    {
        // TODO: update with real values
        float clSpeed = std::clamp(speed, -1.0f, 1.0f);
        return static_cast<uint32_t>(std::abs(clSpeed) * 1000.0);
    }
    void RaeMotor::motorSet(float speed)
    {
        bool _direction = (speed >= 0) ^ reversePhPinLogic_;
        uint32_t speedMil = speedToPWM(speed);
        if (_direction == direction)
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
            direction = _direction;
            if (direction)
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
    }

    void RaeMotor::run()
    {
        _running = true;
        motorThread = std::thread(&RaeMotor::pwmMotor, this);
        encoderThread = std::thread(&RaeMotor::readEncoders, this);
        if (direction)
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
        pwmPin.set_value(0);
        pwmPin.release();
        phPin.set_value(0);
        phPin.release();
        enAPin.release();
        enBPin.release();
    }

}
