#include "control.hpp"
#include <Arduino.h>
#include <math_utility.hpp>

ControlMethod::~ControlMethod() {}

LQRControlMethod::LQRControlMethod(float thetaArmGain, float thetaArmDotGain, float thetaPendulumGain, float thetaPendulumDotGain, float trackingGain) {
    this->thetaArmGain = thetaArmGain;
    this->thetaArmDotGain = thetaArmDotGain;
    this->thetaPendulumGain = thetaPendulumGain;
    this->thetaPendulumDotGain = thetaPendulumDotGain;
    this->trackingGain = trackingGain;
}

float LQRControlMethod::getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle) {
    float output = - thetaPendulumGain * pendulumAngle
                   - thetaPendulumDotGain * pendulumAngularVelocity
                   - thetaArmGain * armAngle
                   - thetaArmDotGain * armAngularVelocity
                   + trackingGain * referenceAngle;
    
    return rad2deg(output) * PI / 7200;
}

LyapunovControlMethod::LyapunovControlMethod(float proportionalGain) {
    this->proportionalGain = proportionalGain;
}

float LyapunovControlMethod::getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle) {
    (void)armAngle;
    (void)armAngularVelocity;
    (void)referenceAngle;

    float cosPendulum = cos(deg2rad(pendulumAngle));
    float pendulumAngularVelocityRadians = deg2rad(pendulumAngularVelocity);
    float energy = 0.5f * 0.0005989206600000001f * (pendulumAngularVelocityRadians) * (pendulumAngularVelocityRadians) + 0.095715f * 9.81f * 0.137f/2 * (-1 + cosPendulum) * 1.93f;
    float output =  proportionalGain * (energy - 0.1241357555f) * sign(pendulumAngularVelocityRadians * cosPendulum);

    return rad2deg(output);
}
