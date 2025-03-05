#include <unity.h>
#include <input_variable.hpp>

const float VALUE_1 = 1;
const float VALUE_2 = 2;
const float VALUE_3 = 3;
const float VALUE_4 = 4;
const float VALUE_5 = 5;
const float VALUES_SET[] = {VALUE_1, VALUE_2, VALUE_3, VALUE_4, VALUE_5};
const int TOTAL_VALUES_SET = 5;
const float CURRENT_TIME_SET = 0;
const float HOLD_TIME_SET = 1;
TimeVaryingInput* timeVaryingInput;

void setUp() {
    timeVaryingInput = new TimeVaryingInput(VALUES_SET, TOTAL_VALUES_SET, CURRENT_TIME_SET, HOLD_TIME_SET);
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


void testTimeVaryingInputUpdates


int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(testTimeVaryingInputReturnsValue);
    RUN_TEST(testTimeVaryingInputReturnsValuesArray);
    RUN_TEST(testTimeVaryingInputReturnsTotalValues);
    RUN_TEST(testTimeVaryingInputReturnsCurrentIndex);
    UNITY_END();   
}