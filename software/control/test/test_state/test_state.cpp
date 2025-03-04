#include <unity.h>
#include <Arduino.h>
#include <state.hpp>



const char* STATE_NAME_SET = "state";
State* state;

bool enteredEnterFunction = false;
bool enteredExitFunction = false;
bool enteredUpdateFunction = false;


void dummyEnterFunction() {
    enteredEnterFunction = true;
}

void dummyExitFunction() { 
    enteredExitFunction = true;
}

void dummyUpdateFunction() {
    enteredUpdateFunction = true;
}   



void setUp() {
    state = new State(&dummyEnterFunction, &dummyExitFunction, &dummyUpdateFunction, STATE_NAME_SET);
    enteredEnterFunction = false;
    enteredExitFunction = false;
    enteredUpdateFunction = false;
}

void tearDown() {
    delete state;
}

void testStateInitialisesAndReturnsMembers(void) {
    void (*onEnterEventFunction)() = state->getOnEnterEventFunction();
    void (*onExitEventFunction)() = state->getOnExitEventFunction();
    void (*duringUpdateEventFunction)() = state->getUpdateEventFunction();
    const char* name = state->getName();

    TEST_ASSERT_EQUAL(&dummyEnterFunction, onEnterEventFunction);
    TEST_ASSERT_EQUAL(&dummyExitFunction, onExitEventFunction);
    TEST_ASSERT_EQUAL(&dummyUpdateFunction, duringUpdateEventFunction);
    TEST_ASSERT_EQUAL_STRING(STATE_NAME_SET, name);
}


void testStateRunsOnEnter(void) {
    state->onEnter();

    TEST_ASSERT_TRUE(enteredEnterFunction);
}


void testStateRunsOnExit(void) {
    state->onExit();

    TEST_ASSERT_TRUE(enteredExitFunction);
}


void testStateRunsUpdate(void) {
    state->update();

    TEST_ASSERT_TRUE(enteredUpdateFunction);
}


int main ( int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(testStateInitialisesAndReturnsMembers);
    RUN_TEST(testStateRunsOnEnter);
    RUN_TEST(testStateRunsOnExit);
    RUN_TEST(testStateRunsUpdate);
    UNITY_END();

    return 0;
}