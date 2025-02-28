#include "rotary_inverted_pendulum_system.hpp"
#include <Arduino.h>

constexpr float SECONDS_PER_MICROSECOND = 1e-6;

RotaryInvertedPendulumSystem::RotaryInvertedPendulumSystem(Actuator* stepperMotor, 
    Sensor* pendulumAngleSensor, Sensor* armAngleSensor,
    ControlMethod* swingUpControlMethod, ControlMethod* balanceControlMethod,
    StateVariable* pendulumAngle, StateVariable* armAngle,
    StateVariable* pendulumAngleRateOfChange, StateVariable* armAngleRateOfChange,
    InputVariable* referenceAngle) {
        this->stepperMotor = stepperMotor;
        this->pendulumAngleSensor = pendulumAngleSensor;
        this->armAngleSensor = armAngleSensor;
        this->swingUpControlMethod = swingUpControlMethod;
        this->balanceControlMethod = balanceControlMethod;
        this->pendulumAngle = pendulumAngle;
        this->armAngle = armAngle;
        this->pendulumAngleRateOfChange = pendulumAngleRateOfChange;
        this->armAngleRateOfChange = armAngleRateOfChange;
        this->referenceAngle = referenceAngle;
        this->stateMachine = nullptr;
        instance = this;
}

void RotaryInvertedPendulumSystem::setStateMachine(StateMachine* stateMachine) {
    this->stateMachine = stateMachine;
}

void RotaryInvertedPendulumSystem::run() {
    stateMachine->update();
}


void RotaryInvertedPendulumSystem::updateStateVariables() {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;

    pendulumAngle->update(pendulumAngleSensor->getData(), currentTime);
    armAngle->update(armAngleSensor->getData(), currentTime);
    pendulumAngleRateOfChange->update(pendulumAngle->getRateOfChange(), currentTime);
    armAngleRateOfChange->update(armAngle->getRateOfChange(), currentTime);
}


void RotaryInvertedPendulumSystem::resetReferenceAngle() {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;
    instance->referenceAngle->reset(currentTime);
}

void RotaryInvertedPendulumSystem::runSwingUpControl() {
    instance->updateStateVariables();
    float controlInput = instance->swingUpControlMethod->getOutput(instance->armAngle->getValue(), instance->armAngleRateOfChange->getValue(), instance->pendulumAngle->getValue(), instance->pendulumAngleRateOfChange->getValue(), instance->referenceAngle->getValue());
    instance->stepperMotor->actuate(controlInput);
}


void RotaryInvertedPendulumSystem::runBalanceControl() {
    instance->updateStateVariables();
    float controlInput = instance->balanceControlMethod->getOutput(instance->armAngle->getValue(), instance->armAngleRateOfChange->getValue(), instance->pendulumAngle->getValue(), instance->pendulumAngleRateOfChange->getValue(), instance->referenceAngle->getValue());
    instance->stepperMotor->actuate(controlInput);
}


void RotaryInvertedPendulumSystem::stop() {
    instance->stepperMotor->stop();
}
