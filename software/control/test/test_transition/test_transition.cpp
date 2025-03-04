#include <unity.h>
#include <transition.hpp>
#include <ArduinoFake.h>
#include "state_interface.hpp"

Transition* transition;

fakeit::Mock<StateInterface> stateInterface;

When(Mock(stateInterface).getName()).thenReturn("state");
When(Mock(stateInterface).getOnEnterEventFunction()).thenReturn(nullptr);
When(Mock(stateInterface).getOnExitEventFunction()).thenReturn(nullptr);
When(Mock(stateInterface).getUpdateEventFunction()).thenReturn(nullptr);

StateInterface mockFromState = stateInterface.get();
StateInterface mockToState = stateInterface.get();

const float DUMMY_FUNCTION_PARAMETER = 10;

bool dummyConditionFunction(float parameter) {
    return parameter > 0;
}

void setUp(void) {
    transition = new Transition(&mockFromState, &mockToState, &dummyConditionFunction, DUMMY_FUNCTION_PARAMETER);
}


void tearDown(void) {
    delete transition;
}


void testTransitionInitialisesAndReturnsMembers(void) {
    bool isConditionTrue = transition->isConditionTrue();
    bool transitionFromState = transition->getTransitionFromState();
    bool transitionToState = transition->getTransitionToState();
    float parameter = transition->getParameter();

    TEST_ASSERT_TRUE(isConditionTrue);
    TEST_ASSERT_EQUAL(&mockFromState, transitionFromState);
    TEST_ASSERT_EQUAL(&mockToState, transitionToState);
    TEST_ASSERT_EQUAL(DUMMY_FUNCTION_PARAMETER, parameter);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    UNITY_END();
}