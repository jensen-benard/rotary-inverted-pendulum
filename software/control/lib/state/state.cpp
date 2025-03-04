#include "state.hpp"

State::State(void (*onEnterEventFunction)(), void (*onExitEventFunction)(), void (*duringUpdateEventFunction)(), const char* name): name(name) {
    this->onEnterEventFunction = onEnterEventFunction;
    this->onExitEventFunction = onExitEventFunction;
    this->duringUpdateEventFunction = duringUpdateEventFunction;
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

const char* State::getName() {
    return this->name;
}

void (*State::getOnEnterEventFunction())() {
    return this->onEnterEventFunction;
}  

void (*State::getOnExitEventFunction())() {
    return this->onExitEventFunction;
}   

void (*State::getUpdateEventFunction())() {
    return this->duringUpdateEventFunction;
}