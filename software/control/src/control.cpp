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

/**
 * Calculates the control output for the rotary inverted pendulum system using
 * Linear Quadratic Regulator (LQR) method.
 * 
 * Each sensor reading is converted to radians, and the output back to degrees for unit consistency.
 * However, mathematically this is redundant since the unit conversion cancels out.
 *
 * @param armAngle The current angle of the arm in degrees.
 * @param armAngularVelocity The current angular velocity of the arm in degrees per second.
 * @param pendulumAngle The current angle of the pendulum in degrees.
 * @param pendulumAngularVelocity The current angular velocity of the pendulum in degrees per second.
 * @param referenceAngle The desired reference angle for the system in degrees.
 * @return The control output in degrees, which will be used to actuate the system.
 */
float LQRControlMethod::getOutput(float armAngle, float armAngularVelocity, float pendulumAngle, float pendulumAngularVelocity, float referenceAngle) {
    
    float output = - thetaPendulumGain * deg2rad(pendulumAngle)
                   - thetaPendulumDotGain * deg2rad(pendulumAngularVelocity)
                   - thetaArmGain * deg2rad(armAngle)
                   - thetaArmDotGain * deg2rad(armAngularVelocity)
                   + trackingGain * deg2rad(referenceAngle);
    return rad2deg(output) / 23;
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
