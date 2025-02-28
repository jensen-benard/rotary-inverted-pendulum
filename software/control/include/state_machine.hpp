#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "state.hpp"
#include "transition.hpp"


class StateMachine {
    public: 
        StateMachine(State* initialState, Transition* allTransitions, int totalTransitions);

        void update();

    private:
        State* currentState;
        Transition* allTransitions;
        int totalTransitions;
        int stateActivity;


};

#endif

