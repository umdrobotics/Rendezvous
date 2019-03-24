//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xger.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef XGER_H
#define XGER_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void xger(int m, int n, double alpha1, int ix0, const emxArray_real_T *y,
                 emxArray_real_T *A, int ia0, int lda);

#endif

//
// File trailer for xger.h
//
// [EOF]
//
