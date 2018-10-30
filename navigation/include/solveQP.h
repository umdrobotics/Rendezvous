//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solveQP.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef SOLVEQP_H
#define SOLVEQP_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void solveQP(double P, double M, const double xk[4], const
                    emxArray_real_T *rp, double q, double k, double Qf, double
                    Qb, emxArray_real_T *x);

#endif

//
// File trailer for solveQP.h
//
// [EOF]
//
