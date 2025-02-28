#include "reference.hpp"
#include <Arduino.h>

constexpr double SECONDS_PER_MICROSECOND = 1e-6;

Reference::Reference(double* trajectoryPtr, int trajectoryLength, int angleHoldTime): TRAJECTORY_LENGTH(trajectoryLength), ANGLE_HOLD_TIME(angleHoldTime) {
    currentAngle = 0;
    previousUpdateTime = 0;
    currentTrajectoryIndex = 0;
    trajectory = trajectoryPtr;
    zeroAngle = 0;
}

void Reference::update() {
    double currentTime = micros() * SECONDS_PER_MICROSECOND;
    if (currentTime - previousUpdateTime > ANGLE_HOLD_TIME) {
        currentAngle = zeroAngle + trajectory[currentTrajectoryIndex];
        updateTrajectoryIndex();
        previousUpdateTime = currentTime;
    }
}

double Reference::getCurrentAngle() {
    return currentAngle;
}

void Reference::updateTrajectoryIndex() {
    currentTrajectoryIndex++;
    if (currentTrajectoryIndex > TRAJECTORY_LENGTH - 1) {
    currentTrajectoryIndex = 0;
    }
}

void Reference::reset(float zeroAngle) {
    this->zeroAngle = zeroAngle;

    currentTrajectoryIndex = 0;
    previousUpdateTime = micros() * SECONDS_PER_MICROSECOND;
}  


