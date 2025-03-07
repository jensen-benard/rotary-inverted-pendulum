#include "input_variable.hpp"


InputVariable::~InputVariable() {}

StaticInput::StaticInput(float initialValue) {
    value = initialValue;
}

void StaticInput::reset(float currentTime) {
    (void)currentTime;
}
void StaticInput::update(float currentTime) {
    (void)currentTime;
}

float StaticInput::getValue() {
    return value;
}

TimeVaryingInput::TimeVaryingInput(const float values[], const int TOTAL_VALUES, float currentTime, const float HOLD_TIME): values(values), TOTAL_VALUES(TOTAL_VALUES), HOLD_TIME(HOLD_TIME) {
    currentIndex = 0;
    previousTime = currentTime;
}

void TimeVaryingInput::reset(float currentTime) {
    currentIndex = 0;
    previousTime = currentTime;
}

void TimeVaryingInput::update(float currentTime) {
    float elapsedTime = currentTime - previousTime;
    if (elapsedTime > HOLD_TIME) {
        currentIndex = (currentIndex + 1) % TOTAL_VALUES;
        previousTime = currentTime;
    }
}

float TimeVaryingInput::getValue() {
    return values[currentIndex];
}

const float* TimeVaryingInput::getValues() {
    return values;
}

int TimeVaryingInput::getTotalValues() {
    return TOTAL_VALUES;
}

int TimeVaryingInput::getCurrentIndex() {
    return currentIndex;
}  