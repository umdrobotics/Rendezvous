//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//
#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
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
