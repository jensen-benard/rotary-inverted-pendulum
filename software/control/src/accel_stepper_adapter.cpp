#include "accel_stepper_adapter.hpp"
#include <Arduino.h>

constexpr float SECONDS_PER_MICROSECOND = 1e-6;

AccelStepperAdapter::AccelStepperAdapter(AccelStepper* stepper, const int MICROSTEPS_PER_DEGREE): MICROSTEPS_PER_DEGREE(MICROSTEPS_PER_DEGREE) {
    this->stepper = stepper;
    inputSpeed = 0;
    lastUpdateTime = 0;
}

double AccelStepperAdapter::getData() {
    double angle = stepper->currentPosition() / MICROSTEPS_PER_DEGREE;
    return angle;
}

void AccelStepperAdapter::actuate(float controlInput) {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;
    float elapsedTime = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;
    
    float inputAccel = controlInput;
    inputSpeed += inputAccel * elapsedTime;

    stepper->setSpeed(inputSpeed * MICROSTEPS_PER_DEGREE);
    stepper->runSpeed();
}   

void AccelStepperAdapter::stop() {
    stepper->stop();
    inputSpeed = 0;
    lastUpdateTime = micros() * SECONDS_PER_MICROSECOND;
}