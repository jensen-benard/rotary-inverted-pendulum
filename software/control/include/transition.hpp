#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include "state.hpp"

class Transition {
    public:
        Transition(State* transitionFromState, State* transitionToState, bool (*conditionFunction)());

        bool isConditionTrue();
        State* getTransitionFromState();
        State* getTransitionToState();

    private:
        State* transitionFromState;
        State* transitionToState;
        bool (*conditionFunction)();
};




#endif
