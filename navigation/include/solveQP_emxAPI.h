//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solveQP_emxAPI.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef SOLVEQP_EMXAPI_H
#define SOLVEQP_EMXAPI_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for solveQP_emxAPI.h
//
// [EOF]
//
