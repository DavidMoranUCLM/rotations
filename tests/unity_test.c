#include "quaternions.h"
#include "unity.h"
#include <math.h>
#include <stdlib.h>

#define NUM 5
#define FLOAT_ERROR 1e-6


void setUp(void){}
void tearDown(void){}

void suiteSetUp(void){}
int suiteTearDown(int num_failures){}

void resetTest(void){}
void verifyTest(void){}

void testQuatAlloc(void){
    gsl_quat_float* q = gsl_quat_float_alloc();

    TEST_ASSERT_NOT_NULL(q);

}

void testQuatCalloc(void){
    gsl_quat_float* q = gsl_quat_float_calloc();

    TEST_ASSERT_NOT_NULL(q);
    for(uint8_t i=0; i<q->size; i++){
        TEST_ASSERT(q->data[i]==0);
    }

}


void testQuatNorm(void){
    gsl_quat_float* q = gsl_quat_float_calloc();
    for(int i=0; i<100; i++){
        float normRef = 0;
        float norm;


        for(int8_t i=0; i<q->size; i++){
            q->data[i] = ((float)rand()*100)/(RAND_MAX);
            normRef += q->data[i]*q->data[i];
        }

        normRef = sqrtf(normRef);

        norm = gsl_quat_float_norm(q);

        TEST_ASSERT_FLOAT_ARRAY_WITHIN(FLOAT_ERROR, &normRef, &norm, 1);
    }

}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(testQuatAlloc);
RUN_TEST(testQuatCalloc);
RUN_TEST(testQuatNorm);
return UNITY_END();
}