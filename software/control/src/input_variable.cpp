#include "input_variable.hpp"


InputVariable::~InputVariable() {}

StaticInput::StaticInput(float initialValue) {
    value = initialValue;
}

void StaticInput::reset(float currentTime) {}
void StaticInput::update(float currentTime) {}

float StaticInput::getValue() {
    return value;
}

TimeVaryingInput::TimeVaryingInput(float* values, int totalValues, float currentTime, const float HOLD_TIME): HOLD_TIME(HOLD_TIME) {
    values = values;
    totalValues = totalValues;
    currentIndex = 0;
    previousTime = currentTime;
}

void TimeVaryingInput::reset(float currentTime) {
    currentIndex = 0;
    previousTime = currentTime;
}

void TimeVaryingInput::update(float currentTime) {
    float elapsedTime = currentTime - previousTime;
    previousTime = currentTime;

    if (elapsedTime > HOLD_TIME) {
        currentIndex = (currentIndex + 1) % totalValues;
    }
}

float TimeVaryingInput::getValue() {
    return values[currentIndex];
}