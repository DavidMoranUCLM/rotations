#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <inttypes.h>
#include <stdlib.h>

#include "gsl/gsl_vector_float.h"

typedef gsl_vector_float gsl_quat_float;

/**
 * @return Pointer to an allocated quat
 */
extern gsl_quat_float* gsl_quat_float_alloc(void);

/**
 * @return Pointer to an allocated quat initialized to 0
 */
extern gsl_quat_float* gsl_quat_float_calloc(void);

/**
 * @param[in] pQ Pointer to the quat to free from heap
 */
extern void gsl_quat_float_free(gsl_quat_float* pQ);

/**
 * @brief Gets the norm of pQ
 * @param pQ Pointer to the quat to calculate the norm of
 * @return Float type with the norm of the quat
 */
extern float gsl_quat_float_norm(gsl_quat_float* pQ);

/**
 * @brief Normalizes pQ
 * @param pQ Pointer to the quat to normalize
 */
extern void gsl_quat_float_normamilize(gsl_quat_float* pQ);

/**
 * @brief Gets the conjugate of pQ
 * @param pQ Pointer to the quat to calculate the conjugate of
 * @return Pointer to the conjugate o pQ
 */
extern gsl_quat_float* gsl_quat_float_conjugate(gsl_quat_float* pQ);

/**
 * @brief Copies the data of pSrc to pDst
 * @param pSrc Pointer to the quat to copy from
 * @param pDst Pointer to the quat to copy to
 */
extern void gsl_quat_float_copy(gsl_quat_float* pSrc, gsl_quat_float* pDst);

/**
 * @brief Performs the product pQ1 pQ2
 * @param pQ1
 * @param pQ2
 * @return Product of pQ1 pQ2
 */
extern gsl_quat_float* gsl_quat_float_product(gsl_quat_float* pQ1,
                                              gsl_quat_float* pQ2);

/**
 * @brief Creates a quat from a Axis vector and a Angle in radians
 * @param pAxis Pointer to the vector to use as an axis
 * @param angleRad Angle in radians
 * @return Pointer to the created quat
 */
extern gsl_quat_float* gsl_quat_float_fromAxis(gsl_vector_float* pAxis,
                                               float angleRad);

#endif