#include "rae_hw/rae_motors.hpp"

namespace rae_hw
{
    RaeMotor::RaeMotor(const std::string &name, const std::string &chipName, int pwmPinNum, int phPinNum, bool reversePhPinLogic)
    {
        pwmPin = pwmPinNum;
        phPin = phPinNum;
        reversePhPinLogic_ = reversePhPinLogic;
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
                write(pwmPin, "1", 1);
            }
            usleep(dutyTrue);
            write(pwmPin, "0", 1);
            usleep((1000 - dutyTrue));
            dutyTrue = dutyTarget;
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
                write(phPin, "1", 1);
            }
            else
            {
                write(phPin, "0", 1);
            }

            // Set speed
            dutyTarget = speedMil;
        }
    }

    void RaeMotor::run()
    {
        _running = true;
        motorThread = std::thread(&RaeMotor::pwmMotor, this);
        if (direction)
        {
            write(phPin, "1", 1);
        }
        else
        {
            write(phPin, "0", 1);
        }
    }

    void RaeMotor::stop()
    {
        _running = false;
        motorThread.join();
    }

}
