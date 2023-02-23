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
    RaeMotor::~RaeMotor(){
        stop();
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
    uint32_t RaeMotor::speedToPWM(float speed){
        // TODO: update with real values
        float clSpeed = std::clamp(speed, -1.0f, 1.0f);
        return static_cast<uint32_t>(std::abs(clSpeed) * 1000.0);
    }
    void RaeMotor::motorSet(float speed)
    {
        bool _direction = speed > 0.0;
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