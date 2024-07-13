#include "unity.h"
#include "rotation_matrix.h"

void setUp(void) {}
void tearDown(void) {}

void suiteSetUp(void) {}
int suiteTearDown(int num_failures) {}

void resetTest(void) {}
void verifyTest(void) {}



void testFail(void) {
    TEST_FAIL();

}


int main(void){
    UNITY_BEGIN();
    testFail;
    UNITY_END();
}