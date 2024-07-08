#include "unity.h"

#include <math.h>
#include <stdlib.h>

#include "quaternions.h"

#define PROD_NORM_ITER 10000
#define FLOAT_ERROR 1e-6

void randQuat(gsl_quat_float* q) {
  for (int8_t i = 0; i < q->size; i++) {
    q->data[i] = ((float)rand() * 100) / (RAND_MAX);
  }
}

void setUp(void) {}
void tearDown(void) {}

void suiteSetUp(void) {}
int suiteTearDown(int num_failures) {}

void resetTest(void) {}
void verifyTest(void) {}

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

void isNormalized(gsl_quat_float* q) {
  float norm = gsl_quat_float_norm(q);
  TEST_ASSERT_EQUAL_FLOAT(1, norm);
}

void testNormalize(void) {
  gsl_quat_float* q = gsl_quat_float_alloc();

  for (int8_t i = 0; i < q->size; i++) {
    q->data[i] = ((float)rand() * 100) / (RAND_MAX);
  }

  gsl_quat_float_normamilize(q);

  isNormalized(q);
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

  TEST_ASSERT_FLOAT_ARRAY_WITHIN(FLOAT_ERROR, q2, q3, 4);
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

  TEST_ASSERT_FLOAT_ARRAY_WITHIN(FLOAT_ERROR, q3Ref->data, q3->data, 4);

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

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(testQuatAlloc);
  RUN_TEST(testQuatCalloc);
  RUN_TEST(testQuatNorm);
  RUN_TEST(testNormalize);
  RUN_TEST(testQuatCopy);
  RUN_TEST(testQuatProd);
  RUN_TEST(testQuatProdNorm);
  return UNITY_END();
}