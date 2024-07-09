#include "quaternions.h"

#include <gsl/gsl_vector_float.h>
#include <math.h>

#define QUAT_SIZE 4
#define NORMALIZE_DELTA (nextafterf(1, 2) - 1)
#define QUAT_MINIMUM_AXIS_NORM 1e-3f

/**
 * Private definitions
 **/

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

gsl_quat_float *gsl_quat_float_conjugate(gsl_quat_float *pQ) {
  gsl_quat_float *pConjQ = gsl_quat_float_alloc();

  pConjQ->data[0] = pQ->data[0];
  pConjQ->data[1] = -pQ->data[1];
  pConjQ->data[2] = -pQ->data[2];
  pConjQ->data[3] = -pQ->data[3];

  return pConjQ;
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

gsl_quat_float *gsl_quat_float_fromAxis(gsl_vector_float *pAxis,
                                        float angleRad) {
  if (pAxis->size != 3) {
    return NULL;
  }

  if (gsl_quat_float_norm(pAxis) < QUAT_MINIMUM_AXIS_NORM){
    return NULL;;
  }

  gsl_quat_float *pQ = gsl_quat_float_alloc();

  float sinHalfAngle = sinf(angleRad / 2);

  pQ->data[0] = cosf(angleRad / 2);
  pQ->data[1] = pAxis->data[0] * sinHalfAngle;
  pQ->data[2] = pAxis->data[1] * sinHalfAngle;
  pQ->data[3] = pAxis->data[2] * sinHalfAngle;

  gsl_quat_float_normamilize(pQ);

  return pQ;
}

gsl_quat_float *gsl_quat_float_fromVector(gsl_vector_float *pVector) {
  if (pVector->size != 3) {
    return NULL;
  }

  gsl_quat_float *pQ = gsl_quat_float_calloc();
  for (uint8_t i = 0; i < pVector->size; i++) {
    pQ->data[i + 1] = pVector->data[i];
  }

  return pQ;
}

/**
 * Private functions definitions
 */
float fastSqrtf(float squaredNorm) {
  return (1 + squaredNorm) / (1 + 0.5 * squaredNorm);
}