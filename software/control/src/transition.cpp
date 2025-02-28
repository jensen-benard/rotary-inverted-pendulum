#include "transition.hpp"

Transition::Transition(State* transitionFromState, State* transitionToState, bool (*conditionFunction)(float), float parameter) {
    this->transitionFromState = transitionFromState;
    this->transitionToState = transitionToState;
    this->conditionFunction = conditionFunction;
    this->parameter = parameter;
}


bool Transition::isConditionTrue() {
    return conditionFunction(parameter);
}


State* Transition::getTransitionFromState() {
    return transitionFromState;
}


State* Transition::getTransitionToState() {
    return transitionToState;
}