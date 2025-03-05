#include <unity.h>
#include <state_machine.hpp>
#include <state.hpp>
#include <transition.hpp>
#include <Arduino.h>

StateMachine* stateMachine;
State* state1;
State* state2;
State* state3;
State* state4;
Transition* transition1;
Transition* transition2;
Transition* transition3;
Transition* transition4;
const int TOTAL_TRANSITIONS = 4;
Transition* allTransitionsExpected[TOTAL_TRANSITIONS];

const float DUMMY_FUNCTION_PARAMETER = 0;

bool dummyConditionFunctionTrue(float parameter) {
    (void)parameter;
    return true;
}

bool dummyConditionFunctionFalse(float parameter) {
    (void)parameter;
    return false;
}

void setUp() {
    state1 = new State(nullptr, nullptr, nullptr, "state1");
    state2 = new State(nullptr, nullptr, nullptr, "state2");
    state3 = new State(nullptr, nullptr, nullptr, "state3");
    state4 = new State(nullptr, nullptr, nullptr, "state4");
    transition1 = new Transition(state1, state2, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition2 = new Transition(state2, state3, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition3 = new Transition(state3, state1, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition4 = new Transition(nullptr, state4, &dummyConditionFunctionFalse, DUMMY_FUNCTION_PARAMETER);
    allTransitionsExpected[0] = transition4;
    allTransitionsExpected[1] = transition1;
    allTransitionsExpected[2] = transition2;
    allTransitionsExpected[3] = transition3;
    stateMachine = new StateMachine(state1, allTransitionsExpected, TOTAL_TRANSITIONS);
}

void setupWithTrueConditionForState4AndOtherStatesConditionFalse() {
    state1 = new State(nullptr, nullptr, nullptr, "state1");
    state2 = new State(nullptr, nullptr, nullptr, "state2");
    state3 = new State(nullptr, nullptr, nullptr, "state3");
    state4 = new State(nullptr, nullptr, nullptr, "state4");
    transition1 = new Transition(state1, state2, &dummyConditionFunctionFalse, DUMMY_FUNCTION_PARAMETER);
    transition2 = new Transition(state2, state3, &dummyConditionFunctionFalse, DUMMY_FUNCTION_PARAMETER);
    transition3 = new Transition(state3, state1, &dummyConditionFunctionFalse, DUMMY_FUNCTION_PARAMETER);
    transition4 = new Transition(nullptr, state4, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    allTransitionsExpected[0] = transition4;
    allTransitionsExpected[1] = transition1;
    allTransitionsExpected[2] = transition2;
    allTransitionsExpected[3] = transition3;
    stateMachine = new StateMachine(state1, allTransitionsExpected, TOTAL_TRANSITIONS);
}


void setupWithTrueConditionAllStates() {
    state1 = new State(nullptr, nullptr, nullptr, "state1");
    state2 = new State(nullptr, nullptr, nullptr, "state2");
    state3 = new State(nullptr, nullptr, nullptr, "state3");
    state4 = new State(nullptr, nullptr, nullptr, "state4");
    transition1 = new Transition(state1, state2, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition2 = new Transition(state2, state3, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition3 = new Transition(state3, state1, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    transition4 = new Transition(nullptr, state4, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
    allTransitionsExpected[0] = transition4;
    allTransitionsExpected[1] = transition1;
    allTransitionsExpected[2] = transition2;
    allTransitionsExpected[3] = transition3;
    stateMachine = new StateMachine(state1, allTransitionsExpected, TOTAL_TRANSITIONS);
}


void tearDown() {
    delete state1;
    delete state2;
    delete state3;
    delete state4;
    delete transition1;
    delete transition2;
    delete transition3;
    delete transition4;
    delete stateMachine;
}


void testStateMachineReturnsCurrentState() {
    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state1, currentState);
}

void testStateMachineReturnsAllTransitions() {
    Transition** transitions = stateMachine->getAllTransitions();
    TEST_ASSERT_EQUAL(allTransitionsExpected, transitions);
}

void testStateMachineReturnsTotalTransitions() {
    int totalTransitions = stateMachine->getTotalTransitions();

    TEST_ASSERT_EQUAL(TOTAL_TRANSITIONS, totalTransitions);
}

void testStateMachineTransitionsFromState1ToState2() {
    stateMachine->update();

    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state2, currentState);
}

void testStateMachineTransitionsFromState2ToState3() {
    stateMachine->update();
    stateMachine->update();

    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state3, currentState);
}

void testStateMachineTransitionsFromState3ToState1() {
    stateMachine->update();
    stateMachine->update();
    stateMachine->update();

    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state1, currentState);
}

void testStateMachineTransitionsFromNullptrToState4() {
    tearDown();
    setupWithTrueConditionForState4AndOtherStatesConditionFalse();

    stateMachine->update();

    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state4, currentState);
}   

void testStateMachineTransitionsFromNullptrToState4WithPriority() {
    tearDown();
    setupWithTrueConditionAllStates();

    stateMachine->update();

    State* currentState = stateMachine->getCurrentState();

    TEST_ASSERT_EQUAL(state4, currentState);
}


int main(int argc, char **argv) {
    (void)argc;
    (void)argv; 

    UNITY_BEGIN();
    RUN_TEST(testStateMachineReturnsCurrentState);
    RUN_TEST(testStateMachineReturnsAllTransitions);
    RUN_TEST(testStateMachineReturnsTotalTransitions);
    RUN_TEST(testStateMachineTransitionsFromState1ToState2);
    RUN_TEST(testStateMachineTransitionsFromState2ToState3);
    RUN_TEST(testStateMachineTransitionsFromState3ToState1);
    RUN_TEST(testStateMachineTransitionsFromNullptrToState4);
    RUN_TEST(testStateMachineTransitionsFromNullptrToState4WithPriority);
    UNITY_END();

    return 0;
}