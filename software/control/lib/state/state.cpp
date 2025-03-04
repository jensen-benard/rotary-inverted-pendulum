#include "state.hpp"

State::State(void (*onEnterEventFunction)(), void (*onExitEventFunction)(), void (*duringUpdateEventFunction)(), char* name) {
    this->onEnterEventFunction = onEnterEventFunction;
    this->onExitEventFunction = onExitEventFunction;
    this->duringUpdateEventFunction = duringUpdateEventFunction;
    this->name = name;
}

void State::onEnter() {
    if (this->onEnterEventFunction != nullptr) {
        this->onEnterEventFunction();
    }
}

void State::onExit() {
    if (this->onExitEventFunction != nullptr) {
        this->onExitEventFunction();
    }
}

void State::update() {
    if (this->duringUpdateEventFunction != nullptr) {
        this->duringUpdateEventFunction();
    }
}

char* State::getName() {
    return this->name;
}