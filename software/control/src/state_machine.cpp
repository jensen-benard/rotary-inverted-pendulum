#include "state_machine.hpp"
#include <Arduino.h>

StateMachine::StateMachine(State* initialState, Transition** allTransitions, int totalTransitions) {
    this->currentState = initialState;
    this->allTransitions = allTransitions;
    this->totalTransitions = totalTransitions;
    
    this->currentState->onEnter();
}

void StateMachine::update() {
    this->currentState->update();

    for (int i = 0; i < this->totalTransitions; i++) {
        Transition* transition = this->allTransitions[i];
        if (transition->getTransitionFromState() == this->currentState || transition->getTransitionFromState() == nullptr) {
            if (transition->isConditionTrue()) {
                this->currentState->onExit();
                this->currentState = transition->getTransitionToState();
                this->currentState->onEnter();
            }
        }
    }
}