#include <unity.h>
#include <math_utility.hpp>


void setUp(void) {
    
}

void tearDown(void) {
    // clean stuff up here
}


void testRad2DegReturnsCorrectValue() {
    float expected = 57.29577951;

    float initialValue = 1;
    float value = rad2deg(initialValue);

    TEST_ASSERT_EQUAL_FLOAT(expected, value);
}


void testDeg2RadReturnsCorrectValue() {
    float expected = 0.01745329252;

    float initialValue = 1;
    float value = deg2rad(initialValue);

    TEST_ASSERT_EQUAL_FLOAT(expected, value);
}


void testSignReturnsPositive() {
    int expected = 1;
    int value = sign(1);

    TEST_ASSERT_EQUAL(expected, value);
}

void testSignReturnsNegative() {
    int expected = -1;
    int value = sign(-1);

    TEST_ASSERT_EQUAL(expected, value);
}
    

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(testRad2DegReturnsCorrectValue);
    RUN_TEST(testDeg2RadReturnsCorrectValue);
    RUN_TEST(testSignReturnsPositive);
    RUN_TEST(testSignReturnsNegative);
    UNITY_END();    

    return 0;
}