#include "rae_hw/rae_motors.hpp"

#include <math.h>

namespace rae_hw

{
using namespace std::chrono_literals;
RaeMotor::RaeMotor(const std::string& name,
                   const std::string& chipName,
                   const std::string& pwmName,
                   int pwmPinNum,
                   int phPinNum,
                   int enA,
                   int enB,
                   float encTicsPerRev,
                   float maxVel,
                   bool reversePhPinLogic,
                   bool closedLoop,
                   PID pid) {
    gpiod::chip chip(chipName);
    pwmPin = pwmPinNum;
    pwmName_ = pwmName;
    phPin = chip.get_line(phPinNum);
    enAPin = chip.get_line(enA);
    enBPin = chip.get_line(enB);
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
    std::string periodPath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/period";
    std::ofstream periodFile(periodPath);
    periodFile << 100000;
    periodFile.close();
}
RaeMotor::~RaeMotor() {
    stop();
}

float RaeMotor::calcSpeed() {
    // Get current position and time different and caluclate the speed
    auto currTime = std::chrono::high_resolution_clock::now();
    float currPos = getPos();
    float timeDiff = std::chrono::duration<float>(currTime - prevVelTime).count();
    {
        std::lock_guard<std::mutex> lck(speedMtx);
        currentSpeed = (currPos - prevPos) / timeDiff;
    }
    prevVelTime = currTime;
    prevPos = currPos;
    return currentSpeed;
}

void RaeMotor::controlSpeed() {
    while(_running) {
        std::string dutyCyclePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/duty_cycle";
        std::ofstream dutyCycleFile(dutyCyclePath);
        bool dir;
        uint32_t dutyCycle;
        float currSpeed = calcSpeed();
        int i_param_counter = 0;
        if(closedLoop_) {
            auto currTime = std::chrono::high_resolution_clock::now();
            float timeDiff = std::chrono::duration<float>(currTime - prevErrorTime).count();
            float error = targetSpeed - currSpeed;
            float eP = error * currPID.P;
            if(i_param_counter == 500) {
                errSum = 0;
                i_param_counter = 0;
            }
            errSum += (error * timeDiff);
            float eI = errSum * currPID.I;
            float eD = (error - prevError) / timeDiff * currPID.D;
            float outSpeed = targetSpeed + eP + eI + eD;
            dutyCycle = speedToPWM(outSpeed);
            dir = (outSpeed >= 0) ^ reversePhPinLogic_;
            prevErrorTime = currTime;
            prevError = error;
            i_param_counter++;
        } else {
            uint32_t dutyCycle = speedToPWM(targetSpeed);
            dir = (targetSpeed >= 0) ^ reversePhPinLogic_;
            if(dutyCycleFile.is_open()) {
                dutyCycleFile << dutyCycle;
                dutyCycleFile.close();
            }
        }

        if(dir == motDirection) {
            // Same direction, no need to switch
        } else {
            // Switch direction
            // First stop
            std::string enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
            std::ofstream enableFile(enablePath);
            if(enableFile.is_open()) {
                enableFile << "0";
                enableFile.close();
            }
            motDirection = dir;
            // Set the direction pin
            if(motDirection) {
                phPin.set_value(1);
            } else {
                phPin.set_value(0);
            }
            enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
            std::ofstream enableFile2(enablePath);
            if(enableFile2.is_open()) {
                enableFile2 << "1";
                enableFile2.close();
            }
        }

        if(dutyCycleFile.is_open()) {
            dutyCycleFile << dutyCycle;
            dutyCycleFile.close();
        }
        usleep(5000);
    }
}

void RaeMotor::setPWM(int duty_cycle) {
    std::string dutyCyclePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/duty_cycle";
    std::ofstream dutyCycleFile(dutyCyclePath);
    dutyCycleFile << duty_cycle;
    dutyCycleFile.close();
    std::string enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
    std::ofstream enableFile(enablePath);
    enableFile << 1;  // Enable PWM
    enableFile.close();
}

void RaeMotor::disablePWM() {
    std::string enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
    std::ofstream enableFile(enablePath);
    enableFile << 0;  // Disable PWM
    enableFile.close();
}
void RaeMotor::readEncoders() {
    while(_running) {
        int currA = enAPin.get_value();
        int currB = enBPin.get_value();
        State currS{currA, currB};
        bool direction = true;  // true is forwards, false is backwards
        int count = prevCount;
        if(currS == Clockwise && currS != prevState) {
            if(prevState == Rest) {
                count++;
                direction = true;
            } else if(prevState == Halfway) {
                count--;
                direction = false;
            } else {
                count = direction ? (count + 2) : (count - 2);
            }
        } else if(currS == Halfway && currS != prevState) {
            if(prevState == Clockwise) {
                count++;
                direction = true;
            } else if(prevState == Counter)

            {
                count--;
                direction = false;
            } else {
                count = direction ? (count + 2) : (count - 2);
            }
        } else if(currS == Counter && currS != prevState) {
            if(prevState == Halfway) {
                count++;
                direction = true;
            } else if(prevState == Rest) {
                count--;
                direction = true;
            } else {
                count = direction ? (count + 2) : (count - 2);
            }
        } else if(currS == Rest && currS != prevState) {
            if(prevState == Counter) {
                count++;
                direction = true;
            } else if(prevState == Clockwise) {
                count--;
                direction = true;
            } else {
                count = direction ? (count + 2) : (count - 2);
            }
        }
        if(count != prevCount) {
            {
                std::lock_guard<std::mutex> lck(posMtx);
                rads = count * encRatio;
            }
            prevCount = count;
        }
        prevState = currS;
        usleep(5);
    }
}

float RaeMotor::getPos() {
    std::lock_guard<std::mutex> lck(posMtx);
    return rads;
}
float RaeMotor::getSpeed() {
    std::lock_guard<std::mutex> lck(speedMtx);
    return currentSpeed;
}

uint32_t RaeMotor::speedToPWM(float speed) {
    // Calculate the duty cycle as a percentage of the maximum period
    float normSpeed = speed / velLim;  // Normalize speed to the range [-1.0, 1.0]
    float clSpeedNorm = std::clamp(normSpeed, -1.0f, 1.0f);
    uint32_t normduty = static_cast<uint32_t>(std::abs(clSpeedNorm * 100000.0f));
    return normduty;
}
void RaeMotor::motorSet(float speed) {
    std::string enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
    std::ofstream enableFile(enablePath);
    enableFile << 1;  // Enable PWM
    enableFile.close();
    targetSpeed = speed;
}

void RaeMotor::run() {
    _running = true;
    std::string enablePath = pwmName_ + "/pwm" + std::to_string(pwmPin) + "/enable";
    std::ofstream enableFile(enablePath);
    enableFile << 1;  // Enable PWM
    enableFile.close();
    encoderThread = std::thread(&RaeMotor::readEncoders, this);
    speedControlThread = std::thread(&RaeMotor::controlSpeed, this);
    if(motDirection) {
        phPin.set_value(1);
    } else {
        phPin.set_value(0);
    }
}

void RaeMotor::stop() {
    _running = false;
    disablePWM();
    encoderThread.join();
    speedControlThread.join();
    phPin.set_value(0);
    phPin.release();
    enAPin.release();
    enBPin.release();
}

}  // namespace rae_hw
