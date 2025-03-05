#include "state_machine.hpp"
#include <Arduino.h>

StateMachine::StateMachine(State* initialState, Transition* allTransitions[], int totalTransitions) {
    currentState = initialState;
    this->allTransitions = allTransitions;
    this->totalTransitions = totalTransitions;

    this->currentState->onEnter();
}

void StateMachine::update() {
    currentState->update();

    for (int i = 0; i < totalTransitions; i++) {
        Transition* transition = allTransitions[i];
        if (transition->getTransitionFromState() == currentState || transition->getTransitionFromState() == nullptr) {
            if (transition->isConditionTrue()) {
                this->currentState->onExit();
                this->currentState = transition->getTransitionToState();
                this->currentState->onEnter();
                break;
            }
        }
    }
}


State* StateMachine::getCurrentState() {
    return this->currentState;
}

Transition** StateMachine::getAllTransitions() {
    return this->allTransitions;
}

int StateMachine::getTotalTransitions() {
    return this->totalTransitions;
}