//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
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
extern void xgemv(int m, int n, const double A_data[], int ia0, int lda, const
                  double x_data[], int ix0, double y_data[]);

#endif

//
// File trailer for xgemv.h
//
// [EOF]
//
