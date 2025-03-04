#include <unity.h>
#include <state_variable.hpp>

const float INITIAL_VALUE_SET = 1;
const float INITIAL_RATE_OF_CHANGE_SET = 2;
const float INITIAL_PREVIOUS_TIME_SET = 3;

StateVariable* stateVariable;

void setUp(void) {
    stateVariable = new StateVariable(INITIAL_VALUE_SET, INITIAL_RATE_OF_CHANGE_SET, INITIAL_PREVIOUS_TIME_SET);
}

void tearDown(void) {
    delete stateVariable;
}


void testStateVariableInitialisesAndReturnsMembers() {
    
    float value = stateVariable->getValue();
    float rateOfChange = stateVariable->getRateOfChange();
    float previousTime = stateVariable->getPreviousTime();
    
    TEST_ASSERT_EQUAL_FLOAT(INITIAL_VALUE_SET, value);
    TEST_ASSERT_EQUAL_FLOAT(INITIAL_RATE_OF_CHANGE_SET, rateOfChange);
    TEST_ASSERT_EQUAL_FLOAT(INITIAL_PREVIOUS_TIME_SET, previousTime);
}


void testStateVariableUpdatesValueAndPreviousTime() {
    
    const float NEW_VALUE_SET = 4;
    const float CURRENT_TIME_SET = 5;
    stateVariable->update(NEW_VALUE_SET, CURRENT_TIME_SET);
    
    float value = stateVariable->getValue();
    float previousTime = stateVariable->getPreviousTime();
    
    TEST_ASSERT_EQUAL_FLOAT(NEW_VALUE_SET, value);
    TEST_ASSERT_EQUAL_FLOAT(CURRENT_TIME_SET, previousTime);
}


void testStateVariableUpdatesRateOfChangeOnce() {
    
    const float NEW_VALUE_SET = 4;
    const float CURRENT_TIME_SET = 5;
    const float EXPECTED_RATE_OF_CHANGE = (NEW_VALUE_SET - INITIAL_VALUE_SET) / (CURRENT_TIME_SET - INITIAL_PREVIOUS_TIME_SET);
    
    stateVariable->update(NEW_VALUE_SET, CURRENT_TIME_SET);
    
    float rateOfChange = stateVariable->getRateOfChange();

    TEST_ASSERT_EQUAL_FLOAT(EXPECTED_RATE_OF_CHANGE, rateOfChange);
}


void testStateVariableUpdatesRateOfChangeMultipleTimes() {
    const float INTERVAL_BETWEEN_UPDATES = 1;
    const float FIRST_NEW_VALUE_SET = 4;
    const float SECOND_NEW_VALUE_SET = 5;
    const float THIRD_NEW_VALUE_SET = 6;
    const float FIRST_CURRENT_TIME_SET = 5;
    const float SECOND_CURRENT_TIME_SET = FIRST_CURRENT_TIME_SET + INTERVAL_BETWEEN_UPDATES;
    const float THIRD_CURRENT_TIME_SET = SECOND_CURRENT_TIME_SET + INTERVAL_BETWEEN_UPDATES;

    const float FIRST_EXPECTED_RATE_OF_CHANGE = (FIRST_NEW_VALUE_SET - INITIAL_VALUE_SET) / (FIRST_CURRENT_TIME_SET - INITIAL_PREVIOUS_TIME_SET);
    const float SECOND_EXPECTED_RATE_OF_CHANGE = (SECOND_NEW_VALUE_SET - FIRST_NEW_VALUE_SET) / (SECOND_CURRENT_TIME_SET - FIRST_CURRENT_TIME_SET);
    const float THIRD_EXPECTED_RATE_OF_CHANGE = (THIRD_NEW_VALUE_SET - SECOND_NEW_VALUE_SET) / (THIRD_CURRENT_TIME_SET - SECOND_CURRENT_TIME_SET);

    stateVariable->update(FIRST_NEW_VALUE_SET, FIRST_CURRENT_TIME_SET);
    float firstRateOfChange = stateVariable->getRateOfChange();
    stateVariable->update(SECOND_NEW_VALUE_SET, SECOND_CURRENT_TIME_SET);
    float secondRateOfChange = stateVariable->getRateOfChange();
    stateVariable->update(THIRD_NEW_VALUE_SET, THIRD_CURRENT_TIME_SET);
    float thirdRateOfChange = stateVariable->getRateOfChange();
    

    TEST_ASSERT_EQUAL_FLOAT(FIRST_EXPECTED_RATE_OF_CHANGE, firstRateOfChange);
    TEST_ASSERT_EQUAL_FLOAT(SECOND_EXPECTED_RATE_OF_CHANGE, secondRateOfChange);
    TEST_ASSERT_EQUAL_FLOAT(THIRD_EXPECTED_RATE_OF_CHANGE, thirdRateOfChange);
}


int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(testStateVariableInitialisesAndReturnsMembers);
    RUN_TEST(testStateVariableUpdatesValueAndPreviousTime);
    RUN_TEST(testStateVariableUpdatesRateOfChangeOnce);
    RUN_TEST(testStateVariableUpdatesRateOfChangeMultipleTimes);

    UNITY_END();

    return 0;
}
