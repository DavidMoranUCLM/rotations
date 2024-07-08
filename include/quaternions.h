#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include<inttypes.h>
#include<stdlib.h>
#include"gsl/gsl_vector_float.h"


typedef gsl_vector_float gsl_quat_float;


extern gsl_quat_float* gsl_quat_float_alloc(void);

extern gsl_quat_float* gsl_quat_float_calloc(void);

extern void gsl_quat_float_free(gsl_quat_float*);

extern float gsl_quat_float_norm(gsl_quat_float*);
extern void gsl_quat_float_normamilize(gsl_quat_float*);

extern gsl_quat_float* gsl_quat_float_fromAxis(gsl_vector_float*, float);
extern gsl_quat_float* gsl_quat_float_product(gsl_quat_float*,gsl_quat_float*);


#endif