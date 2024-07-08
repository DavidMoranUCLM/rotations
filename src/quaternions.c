#include "quaternions.h"
#include <gsl/gsl_vector_float.h>
#include <math.h>


#define QUAT_SIZE 4
#define NORMALIZE_DELTA (1-nextafterf(1,2))

enum QUAT_PROD_E{
    QUAT_NEG_K = -4,
    QUAT_NEG_J = -3,
    QUAT_NEG_I = -2,
    QUAT_NEG_1 = -1,
    QUAT_ZERO = 0,
    QUAT_1 = 1,
    QUAT_I = 2,
    QUAT_J = 3,
    QUAT_K = 4,
};

float fastSqrtf(float);


gsl_quat_float* gsl_quat_float_alloc(void){
    return (gsl_quat_float*)gsl_vector_float_alloc(QUAT_SIZE);
}


gsl_quat_float* gsl_quat_float_calloc(void){
    return (gsl_quat_float*)gsl_vector_float_calloc(QUAT_SIZE);
}

void gsl_quat_free(gsl_quat_float* pQuat){
    gsl_vector_float_free((gsl_vector_float*)pQuat);
}



float fastSqrtf(float squaredNorm){
    return (1+squaredNorm)/(1+0.5*squaredNorm);
}

float gsl_quat_float_norm(gsl_quat_float* q){
    float norm = 0;
    
    for(int8_t i=0; i<q->size; i++){
        norm += q->data[i]*q->data[i];
    }

    if(fabs(norm-1) > NORMALIZE_DELTA){
        return sqrtf(norm);
    }

    return fastSqrtf(norm);

}


float gsl_quat_float_product(gsl_quat_float* q1,gsl_quat_float* q2){

}