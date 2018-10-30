//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solveQP_emxutil.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef SOLVEQP_EMXUTIL_H
#define SOLVEQP_EMXUTIL_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_int16_T(emxArray_int16_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T1(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T1(emxArray_real_T *emxArray, int oldNumel);
extern void emxFreeMatrix_cell_wrap_0(cell_wrap_0 pMatrix[2]);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_int16_T(emxArray_int16_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInitMatrix_cell_wrap_0(cell_wrap_0 pMatrix[2]);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_int16_T(emxArray_int16_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T1(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_real_T1(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for solveQP_emxutil.h
//
// [EOF]
//
