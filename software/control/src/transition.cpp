#include "transition.hpp"

Transition::Transition(State* transitionFromState, State* transitionToState, bool (*conditionFunction)()) {
    this->transitionFromState = transitionFromState;
    this->transitionToState = transitionToState;
    this->conditionFunction = conditionFunction;
}


bool Transition::isConditionTrue() {
    return this->conditionFunction();
}


State* Transition::getTransitionFromState() {
    return this->transitionFromState;
}


State* Transition::getTransitionToState() {
    return this->transitionToState;
}