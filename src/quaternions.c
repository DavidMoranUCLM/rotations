#include "quaternions.h"

#include <gsl/gsl_vector_float.h>
#include <math.h>

#define QUAT_SIZE 4
#define NORMALIZE_DELTA (nextafterf(1, 2) - 1)

/**
 * Private definitions
 **/

enum QUAT_PROD_E { QUAT_1 = 0, QUAT_I, QUAT_J, QUAT_K, QUAT_MAX };

const int8_t quatProdTable[QUAT_MAX][QUAT_MAX] = {
    {QUAT_1, QUAT_I, QUAT_J, QUAT_K},
    {QUAT_I, -QUAT_1, QUAT_K, -QUAT_J},
    {QUAT_J, -QUAT_K, -QUAT_1, QUAT_I},
    {QUAT_K, QUAT_J, -QUAT_I, -QUAT_1},
};

/**
 * Private functions declarations
 */
float fastSqrtf(float);

/**
 * Public functions definitions
 */
gsl_quat_float *gsl_quat_float_alloc(void) {
  return (gsl_quat_float *)gsl_vector_float_alloc(QUAT_SIZE);
}

gsl_quat_float *gsl_quat_float_calloc(void) {
  return (gsl_quat_float *)gsl_vector_float_calloc(QUAT_SIZE);
}

void gsl_quat_float_free(gsl_quat_float *pQ) {
  gsl_vector_float_free((gsl_vector_float *)pQ);
}

float gsl_quat_float_norm(gsl_quat_float *q) {
  float norm = 0;

  for (int8_t i = 0; i < q->size; i++) {
    norm += q->data[i] * q->data[i];
  }

  if (fabs(norm - 1) > NORMALIZE_DELTA) {
    return sqrtf(norm);
  }
  return fastSqrtf(norm - 1);
}

void gsl_quat_float_normamilize(gsl_quat_float *q) {
  float norm = gsl_quat_float_norm(q);
  gsl_vector_float_scale(q, 1 / norm);
}

void gsl_quat_float_copy(gsl_quat_float *qSrc, gsl_quat_float *qDst) {
  for (uint8_t i = 0; i < QUAT_SIZE; i++) {
    qDst->data[i] = qSrc->data[i];
  }
}

gsl_quat_float *gsl_quat_float_product(gsl_quat_float *q1, gsl_quat_float *q2) {
  gsl_quat_float *q3 = gsl_quat_float_alloc();

  q3->data[0] = q1->data[0] * q2->data[0] - q1->data[1] * q2->data[1] -
                q1->data[2] * q2->data[2] - q1->data[3] * q2->data[3];
  q3->data[1] = q1->data[0] * q2->data[1] + q1->data[1] * q2->data[0] +
                q1->data[2] * q2->data[3] - q1->data[3] * q2->data[2];
  q3->data[2] = q1->data[0] * q2->data[2] - q1->data[1] * q2->data[3] +
                q1->data[2] * q2->data[0] + q1->data[3] * q2->data[1];
  q3->data[3] = q1->data[0] * q2->data[3] + q1->data[1] * q2->data[2] -
                q1->data[2] * q2->data[1] + q1->data[3] * q2->data[0];

  return q3;
}

/**
 * Private functions definitions
 */
float fastSqrtf(float squaredNorm) {
  return (1 + squaredNorm) / (1 + 0.5 * squaredNorm);
}