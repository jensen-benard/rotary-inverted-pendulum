#include "control.hpp"
#include <Arduino.h>
#include "math_utility.hpp"

ControlMethod::~ControlMethod() {}

LQRControlMethod::LQRControlMethod(double thetaArmGain, double thetaArmDotGain, double thetaPendulumGain, double thetaPendulumDotGain, double trackingGain) {
    this->thetaArmGain = thetaArmGain;
    this->thetaArmDotGain = thetaArmDotGain;
    this->thetaPendulumGain = thetaPendulumGain;
    this->thetaPendulumDotGain = thetaPendulumDotGain;
    this->trackingGain = trackingGain;
}

double LQRControlMethod::getOutput(double armAngle, double armAngularVelocity, double pendulumAngle, double pendulumAngularVelocity, double referenceAngle) {
    double output = - thetaPendulumGain * deg2rad(pendulumAngle)
                    - thetaPendulumDotGain * deg2rad(pendulumAngularVelocity)
                    - thetaArmGain * deg2rad(armAngle)
                    - thetaArmDotGain * deg2rad(armAngularVelocity)
                    + trackingGain * deg2rad(referenceAngle);

    return rad2deg(output);
}

LyapunovControlMethod::LyapunovControlMethod(double proportionalGain) {
    this->proportionalGain = proportionalGain;
}

double LyapunovControlMethod::getOutput(double armAngle, double armAngularVelocity, double pendulumAngle, double pendulumAngularVelocity, double referenceAngle) {
    float cosPendulum = cos(deg2rad(pendulumAngle));
    float pendulumAngularVelocityRadians = deg2rad(pendulumAngularVelocity);
    float energy = 0.5 * 0.0005989206600000001 * (pendulumAngularVelocityRadians) * (pendulumAngularVelocityRadians) + 0.095715 * 9.81 * 0.137/2 * (-1 + cosPendulum) * 1.93;
    float controlInput =  proportionalGain * (energy - 0.1241357555) * sign(pendulumAngularVelocityRadians * cosPendulum);

    return rad2deg(controlInput);
}