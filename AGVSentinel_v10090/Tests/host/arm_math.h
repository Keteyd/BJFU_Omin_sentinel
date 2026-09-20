#ifndef HOST_ARM_MATH_H
#define HOST_ARM_MATH_H

#include <stdint.h>

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} arm_matrix_instance_f32;

#endif
