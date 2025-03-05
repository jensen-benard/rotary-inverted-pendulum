#include <unity.h>
#include <transition.hpp>
#include <state.hpp>

State* dummyFromState;
State* dummyToState;

Transition* transition;


const float DUMMY_FUNCTION_PARAMETER = 10;

bool dummyConditionFunctionTrue(float parameter) {
    return parameter > 0;
}

void setUp() {
    dummyFromState = new State(nullptr, nullptr, nullptr, "dummyFromState");
    dummyToState = new State(nullptr, nullptr, nullptr, "dummyToState");
    transition = new Transition(dummyFromState, dummyToState, &dummyConditionFunctionTrue, DUMMY_FUNCTION_PARAMETER);
}


void tearDown() {
    delete dummyFromState;
    delete dummyToState;
    delete transition;
}


void testTransitionReturnsParameter() {
    float parameter = transition->getParameter();

    TEST_ASSERT_EQUAL(DUMMY_FUNCTION_PARAMETER, parameter);
}


void testTransitionConditionReturnsTrue() {
    bool isConditionTrue = transition->isConditionTrue();

    TEST_ASSERT_TRUE(isConditionTrue);
}


void testTransitionReturnsFromState() {
    State* transitionFromState = transition->getTransitionFromState();

    TEST_ASSERT_EQUAL(dummyFromState, transitionFromState);  
}


void testTransitionReturnsToState() {
    State* transitionToState = transition->getTransitionToState();

    TEST_ASSERT_EQUAL(dummyToState, transitionToState);  
}



int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(testTransitionReturnsParameter);
    RUN_TEST(testTransitionConditionReturnsTrue);
    RUN_TEST(testTransitionReturnsFromState);
    RUN_TEST(testTransitionReturnsToState);
    UNITY_END();
}