#include "state_variable.hpp"
#include <Arduino.h>
StateVariable::StateVariable(float initialValue, float initialRateOfChange, float currentTime) {
    value = initialValue;
    rateOfChange = initialRateOfChange;
    previousTime = currentTime;
}

void StateVariable::update(float newValue, float currentTime) {
    float elapsedTime = currentTime - previousTime;
    rateOfChange = (newValue - value) / elapsedTime;
    value = newValue;
    previousTime = currentTime;
}

float StateVariable::getValue() {
    return value;
}

float StateVariable::getRateOfChange() {
    return rateOfChange;
}