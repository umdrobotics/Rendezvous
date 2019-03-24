//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void xgemv(int m, int n, const emxArray_real_T *A, int ia0, int lda,
                  const emxArray_real_T *x, int ix0, emxArray_real_T *y);

#endif

//
// File trailer for xgemv.h
//
// [EOF]
//
