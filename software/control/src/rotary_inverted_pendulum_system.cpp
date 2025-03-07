#include "rotary_inverted_pendulum_system.hpp"
#include <Arduino.h>

constexpr float SECONDS_PER_MICROSECOND = 1e-6;

RotaryInvertedPendulumSystem* RotaryInvertedPendulumSystem::instance = nullptr;

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
    instance->stateMachine = stateMachine;
}

void RotaryInvertedPendulumSystem::run() {
    instance->updateStateVariables();
    instance->stateMachine->update();

}


void RotaryInvertedPendulumSystem::updateStateVariables() {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;
    
    instance->pendulumAngle->update(pendulumAngleSensor->getData(), currentTime);
    instance->armAngle->update(armAngleSensor->getData(), currentTime);
    instance->pendulumAngleRateOfChange->update(pendulumAngle->getRateOfChange(), currentTime);
    instance->armAngleRateOfChange->update(armAngle->getRateOfChange(), currentTime);
}


void RotaryInvertedPendulumSystem::reset() {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;
    instance->referenceAngle->reset(currentTime);
    instance->stepperMotor->stop();
}

void RotaryInvertedPendulumSystem::runSwingUpControl() {
    float armAngle = instance->armAngle->getValue();
    float armAngleRateOfChange = instance->armAngleRateOfChange->getValue();
    float pendulumAngle = instance->pendulumAngle->getValue();
    float pendulumAngleRateOfChange = instance->pendulumAngleRateOfChange->getValue();
    float referenceAngle = instance->referenceAngle->getValue();

    float controlInput = instance->swingUpControlMethod->getOutput(armAngle, armAngleRateOfChange, pendulumAngle, pendulumAngleRateOfChange, referenceAngle);
    instance->stepperMotor->actuate(controlInput);
}


void RotaryInvertedPendulumSystem::runBalanceControl() {
    float currentTime = micros() * SECONDS_PER_MICROSECOND;
    instance->referenceAngle->update(currentTime);

    float armAngle = instance->armAngle->getValue();
    float armAngleRateOfChange = instance->armAngleRateOfChange->getValue();
    float pendulumAngle = instance->pendulumAngle->getValue();
    float pendulumAngleRateOfChange = instance->pendulumAngleRateOfChange->getValue();
    float referenceAngle = instance->referenceAngle->getValue();

    float controlInput = instance->balanceControlMethod->getOutput(armAngle, armAngleRateOfChange, pendulumAngle, pendulumAngleRateOfChange, referenceAngle);
    instance->stepperMotor->actuate(controlInput);
}


void RotaryInvertedPendulumSystem::stop() {
    instance->stepperMotor->stop();
    while(true) {
        continue;
    }
}

bool RotaryInvertedPendulumSystem::swingUpCondition(float swingUpTriggerAngle) {
    return abs(instance->pendulumAngle->getValue()) > swingUpTriggerAngle;
}

bool RotaryInvertedPendulumSystem::balanceCondition(float balanceTriggerAngle) {
    return abs(instance->pendulumAngle->getValue()) < balanceTriggerAngle;
}

bool RotaryInvertedPendulumSystem::emergencyStopCondition(float armAngleLimit) {
    return abs(instance->armAngle->getValue()) > armAngleLimit;
}