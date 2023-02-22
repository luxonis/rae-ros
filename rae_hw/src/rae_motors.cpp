#include "rae_hw/rae_motors.hpp"

namespace rae_hw
{
    RaeMotor::RaeMotor(const std::string name, int enPinNum, int phPinNum)
    {
        gpiod::chip chip("gpiochip0");
        enPin = chip.get_line(enPinNum);
        phPin = chip.get_line(phPinNum);
        enPin.request({name+"_en", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
        phPin.request({name+"_ph", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
    }
    void RaeMotor::pwmMotor(){
        while (_running)
        {
            if (dutyTrue)
            {
                enPin.set_value(1);
            }
            usleep(dutyTrue);
            enPin.set_value(0);
            usleep((1000 - dutyTrue));
            dutyTrue = dutyTarget;
        }
    }
    void RaeMotor::motorSet(float speed)
    {

        bool _direction = speed > 0.0;
        float clSpeed = std::clamp(speed, 0.0f, 1.0f);
        // printf("Setting motor with, to %f speed and %d direction\n", speed, _direction);
        uint32_t speedMil = static_cast<uint32_t>(std::abs(clSpeed) * 1000.0);
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
        enPin.set_value(0);
        enPin.release();
        phPin.set_value(0);
        phPin.release();
    }


}