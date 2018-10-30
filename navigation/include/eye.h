//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//
#ifndef EYE_H
#define EYE_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "solveQP_types.h"

// Function Declarations
extern void b_eye(double I[16]);
extern void c_eye(double varargin_1, double varargin_2, emxArray_real_T *I);
extern void eye(double varargin_1, emxArray_real_T *I);

#endif

//
// File trailer for eye.h
//
// [EOF]
//
