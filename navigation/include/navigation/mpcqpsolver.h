//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpcqpsolver.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
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
extern void mpcqpsolver(const double Linv_data[], const int Linv_size[2], const
  double f[10], const double A[680], const double b[68], double x_data[], int
  x_size[1], double *status);

#endif

//
// File trailer for mpcqpsolver.h
//
// [EOF]
//
