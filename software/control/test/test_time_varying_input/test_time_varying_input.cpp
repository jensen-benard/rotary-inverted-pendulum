#include <unity.h>
#include <input_variable.hpp>

const float VALUE_1 = 1;
const float VALUE_2 = 2;
const float VALUE_3 = 3;
const float VALUE_4 = 4;
const float VALUE_5 = 5;
const float VALUES_SET[] = {VALUE_1, VALUE_2, VALUE_3, VALUE_4, VALUE_5};
const int TOTAL_VALUES_SET = 5;
const float INITIAL_TIME_SET = 0;
const float HOLD_TIME_SET = 1;
TimeVaryingInput* timeVaryingInput;

void setUp() {
    timeVaryingInput = new TimeVaryingInput(VALUES_SET, TOTAL_VALUES_SET, INITIAL_TIME_SET, HOLD_TIME_SET);
}


void tearDown() {
    delete timeVaryingInput;
}


void testTimeVaryingInputReturnsValue() {
    float value = timeVaryingInput->getValue();

    TEST_ASSERT_EQUAL_FLOAT(VALUE_1, value);
}

void testTimeVaryingInputReturnsValuesArray() {
    const float* values = timeVaryingInput->getValues();

    TEST_ASSERT_EQUAL_FLOAT_ARRAY(VALUES_SET, values, TOTAL_VALUES_SET);
}

void testTimeVaryingInputReturnsTotalValues() {
    int totalValues = timeVaryingInput->getTotalValues();

    TEST_ASSERT_EQUAL(TOTAL_VALUES_SET, totalValues);
}


void testTimeVaryingInputReturnsCurrentIndex() {
    int currentIndex = timeVaryingInput->getCurrentIndex();

    TEST_ASSERT_EQUAL(0, currentIndex);
}


void testTimeVaryingInputDoesNotUpdateBeforeHoldTime() {
    timeVaryingInput->update(INITIAL_TIME_SET + 0.5);

    int value = timeVaryingInput->getValue();

    TEST_ASSERT_EQUAL(VALUE_1, value);
}

void testTimeVaryingInputUpdatesAfterHoldTime() {
    float timeOne = INITIAL_TIME_SET + HOLD_TIME_SET + 0.1;

    timeVaryingInput->update(timeOne);

    int value = timeVaryingInput->getValue();

    TEST_ASSERT_EQUAL(VALUE_2, value);
}

void testTimeVaryingInputLoopsBackToFirstValue() {
    float timeOne = INITIAL_TIME_SET + HOLD_TIME_SET + 0.1;
    float timeTwo = INITIAL_TIME_SET + 2 * HOLD_TIME_SET + 0.2;
    float timeThree = INITIAL_TIME_SET + 3 * HOLD_TIME_SET + 0.3;
    float timeFour = INITIAL_TIME_SET + 4 * HOLD_TIME_SET + 0.4;
    float timeFive = INITIAL_TIME_SET + 5 * HOLD_TIME_SET + 0.5;

    timeVaryingInput->update(timeOne);
    timeVaryingInput->update(timeTwo);
    timeVaryingInput->update(timeThree);
    timeVaryingInput->update(timeFour);
    timeVaryingInput->update(timeFive);

    int value = timeVaryingInput->getValue();

    TEST_ASSERT_EQUAL(VALUE_1, value);
}


void testTimeVaryingInputResets() {
    float timeOne = INITIAL_TIME_SET + HOLD_TIME_SET + 0.1;

    timeVaryingInput->update(timeOne);
    timeVaryingInput->reset(timeOne);

    int value = timeVaryingInput->getValue();

    TEST_ASSERT_EQUAL(VALUE_1, value);

}
int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(testTimeVaryingInputReturnsValue);
    RUN_TEST(testTimeVaryingInputReturnsValuesArray);
    RUN_TEST(testTimeVaryingInputReturnsTotalValues);
    RUN_TEST(testTimeVaryingInputReturnsCurrentIndex);
    RUN_TEST(testTimeVaryingInputDoesNotUpdateBeforeHoldTime);
    RUN_TEST(testTimeVaryingInputUpdatesAfterHoldTime);
    RUN_TEST(testTimeVaryingInputLoopsBackToFirstValue);
    RUN_TEST(testTimeVaryingInputResets);
    UNITY_END();   
}