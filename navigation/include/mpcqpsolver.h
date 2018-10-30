//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpcqpsolver.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef MPCQPSOLVER_H
#define MPCQPSOLVER_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void mpcqpsolver(const emxArray_real_T *Linv, const emxArray_real_T *f,
  const emxArray_real_T *A, const emxArray_real_T *b, const emxArray_boolean_T
  *iA0, emxArray_real_T *x, double *status);

#endif

//
// File trailer for mpcqpsolver.h
//
// [EOF]
//
