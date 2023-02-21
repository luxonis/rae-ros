#include "rae_hw/rae_motors.hpp"

namespace rae_hw{
    RaeMotor::RaeMotor(int _enPin, int _phPin) {
        enPin = _enPin;
        phPin = _phPin;
    }
    void RaeMotor::setSpeed(float _speed, bool _direction) {
        if(_speed > 1.0 || _speed < 0.0) {
            printf("Warning: Motor speed should be between 0 and 1, it is %f.\n", _speed);
        }
        float clSpeed = std::clamp(_speed, 0.0f, 1.0f);
        printf("Setting motor with %d phase, to %f speed and %d direction\n", phPin, _speed, _direction);
        uint32_t speedMil = static_cast<uint32_t>(std::abs(_speed) * 1000.0);
        if(_direction == direction) {
            dutyTarget = speedMil;
        } else {
            // Switch direction
            // First stop
            dutyTarget = 0;
            while(dutyTrue != 0) {
                usleep(100);
            }
            direction = _direction;
            if(direction) {
                write(phPin, "1", 1);
            } else {
                write(phPin, "0", 1);
            }

            // Set speed
            dutyTarget = speedMil;
        }
    }
}