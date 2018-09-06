//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solveQP_types.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//
#ifndef SOLVEQP_TYPES_H
#define SOLVEQP_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T
#endif

//
// File trailer for solveQP_types.h
//
// [EOF]
//
