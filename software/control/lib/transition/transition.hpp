#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include "state.hpp"

class Transition {
    public:
        Transition(State* transitionFromState, State* transitionToState, bool (*conditionFunction)(float), float parameter);

        bool isConditionTrue();
        State* getTransitionFromState();
        State* getTransitionToState();
        float getParameter();

    private:
        State* transitionFromState;
        State* transitionToState;
        bool (*conditionFunction)(float);
        float parameter;
};




#endif
