#include "unity.h"

#include <math.h>
#include <stdlib.h>

#include "quaternions.h"
#include "gsl/gsl_math.h"


#define PROD_NORM_ITER 10000
#define FLOAT_ERROR 1e-6

#define TEST_ASSERT_QUAT(q1,q2) TEST_ASSERT_EQUAL_FLOAT_ARRAY((q1->data),(q2->data),(4))

void setUp(void) {}
void tearDown(void) {}

void suiteSetUp(void) {}
int suiteTearDown(int num_failures) {}

void resetTest(void) {}
void verifyTest(void) {}



void randQuat(gsl_quat_float* q) {
  for (int8_t i = 0; i < q->size; i++) {
    q->data[i] = ((float)rand() * 100) / (RAND_MAX);
  }
}

void isNormalized(gsl_quat_float* q) {
  float norm = gsl_quat_float_norm(q);
  TEST_ASSERT_EQUAL_FLOAT(1, norm);
}


void testQuatAlloc(void) {
  gsl_quat_float* q = gsl_quat_float_alloc();

  TEST_ASSERT_NOT_NULL(q);

  gsl_quat_float_free(q);
}

void testQuatCalloc(void) {
  gsl_quat_float* q = gsl_quat_float_calloc();

  TEST_ASSERT_NOT_NULL(q);
  for (uint8_t i = 0; i < q->size; i++) {
    TEST_ASSERT(q->data[i] == 0);
  }

  gsl_quat_float_free(q);
}

void testQuatNorm(void) {
  gsl_quat_float* q = gsl_quat_float_calloc();
  for (int i = 0; i < 100; i++) {
    float normRef = 0;
    float norm;

    randQuat(q);

    for (int8_t i = 0; i < q->size; i++) {
      normRef += q->data[i] * q->data[i];
    }

    normRef = sqrtf(normRef);

    norm = gsl_quat_float_norm(q);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(FLOAT_ERROR, &normRef, &norm, 1);
  }

  gsl_quat_float_free(q);
}

void testQuatNormalize(void) {
  gsl_quat_float* q = gsl_quat_float_alloc();

  for (int8_t i = 0; i < q->size; i++) {
    q->data[i] = ((float)rand() * 100) / (RAND_MAX);
  }

  gsl_quat_float_normamilize(q);

  isNormalized(q);
}

void testQuatConjugate(void){
  gsl_quat_float* pQ1 = gsl_quat_float_alloc();
  gsl_quat_float* pQ1Conj = gsl_quat_float_alloc();
  gsl_quat_float* pQ1ConjRef = gsl_quat_float_alloc();

  randQuat(pQ1);

  pQ1ConjRef->data[0] = pQ1->data[0];
  pQ1ConjRef->data[1] = -pQ1->data[1];
  pQ1ConjRef->data[2] = -pQ1->data[2];
  pQ1ConjRef->data[3] = -pQ1->data[3];

  pQ1Conj = gsl_quat_float_conjugate(pQ1);

  TEST_ASSERT_QUAT(pQ1ConjRef, pQ1Conj);

  gsl_quat_float_free(pQ1);
  gsl_quat_float_free(pQ1Conj);
  gsl_quat_float_free(pQ1ConjRef);
}

void testQuatCopy(void) {
  gsl_quat_float* q1 = gsl_quat_float_alloc();
  gsl_quat_float* q2 = gsl_quat_float_alloc();
  gsl_quat_float* q3 = gsl_quat_float_alloc();

  randQuat(q1);
  randQuat(q2);
  randQuat(q3);

  gsl_quat_float_copy(q1, q2);
  gsl_quat_float_copy(q1, q3);

  TEST_ASSERT_QUAT(q2, q3);
}

void testQuatProd(void) {
  gsl_quat_float* q1 = gsl_quat_float_alloc();
  gsl_quat_float* q2 = gsl_quat_float_alloc();
  gsl_quat_float* q3Ref = gsl_quat_float_alloc();

  for (uint8_t i = 0; i < q1->size; i++) {
    q1->data[i] = i + 1;
    q2->data[i] = i + 1;
  }

  q3Ref->data[0] = -28;
  q3Ref->data[1] = 4;
  q3Ref->data[2] = 6;
  q3Ref->data[3] = 8;

  gsl_quat_float* q3 = gsl_quat_float_product(q1, q2);

  TEST_ASSERT_QUAT(q3Ref, q3);

  gsl_quat_float_free(q1);
  gsl_quat_float_free(q2);
  gsl_quat_float_free(q3);
  gsl_quat_float_free(q3Ref);
}

void testQuatProdNorm(void) {
  gsl_quat_float* q1 = gsl_quat_float_alloc();
  gsl_quat_float* q2 = gsl_quat_float_alloc();
  gsl_quat_float* q3;

  randQuat(q1);
  randQuat(q2);

  gsl_quat_float_normamilize(q1);
  gsl_quat_float_normamilize(q2);

  for (int i = 0; i < PROD_NORM_ITER; i++) {
    q3 = gsl_quat_float_product(q1, q2);
    gsl_quat_float_copy(q3, q2);
    gsl_quat_float_normamilize(q2);
    isNormalized(q2);
  }

  gsl_quat_float_free(q1);
  gsl_quat_float_free(q2);
  gsl_quat_float_free(q3);
}

void testQuatFromAxis(void){
  gsl_quat_float* pQ1 = gsl_quat_float_calloc();
  gsl_quat_float* pQ1Ref = gsl_quat_float_calloc();

  gsl_vector_float* pAxis = gsl_vector_float_calloc(3);

  float angle = M_PI/4;
  pAxis->data[0] = 0;
  pAxis->data[1] = 0;
  pAxis->data[2] = 1;

  pQ1Ref->data[0] = 0.9238795;
  pQ1Ref->data[1] = 0;
  pQ1Ref->data[2] = 0;
  pQ1Ref->data[3] = 0.3826835;

  pQ1 = gsl_quat_float_fromAxis(pAxis, angle);

  TEST_ASSERT_QUAT(pQ1Ref, pQ1);

  gsl_quat_float_free(pQ1);
  gsl_quat_float_free(pQ1Ref);
  gsl_vector_float_free(pAxis);
}



int main(void) {
  UNITY_BEGIN();
  RUN_TEST(testQuatAlloc);
  RUN_TEST(testQuatCalloc);
  RUN_TEST(testQuatNorm);
  RUN_TEST(testQuatNormalize);
  RUN_TEST(testQuatConjugate);
  RUN_TEST(testQuatCopy);
  RUN_TEST(testQuatProd);
  RUN_TEST(testQuatProdNorm);
  RUN_TEST(testQuatFromAxis);
  return UNITY_END();
}