//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "eye.h"
#include "solveQP_emxutil.h"

// Function Definitions

//
// Arguments    : double I[16]
// Return Type  : void
//
void b_eye(double I[16])
{
  int k;
  memset(&I[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    I[k + (k << 2)] = 1.0;
  }
}

//
// Arguments    : double varargin_1
//                double varargin_2
//                emxArray_real_T *I
// Return Type  : void
//
void c_eye(double varargin_1, double varargin_2, emxArray_real_T *I)
{
  int m;
  double t;
  int d;
  int i4;
  if (varargin_1 < 0.0) {
    m = 0;
  } else {
    m = (int)varargin_1;
  }

  if (varargin_2 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_2;
  }

  if (m <= (int)t) {
    d = m;
  } else {
    d = (int)t;
  }

  i4 = I->size[0] * I->size[1];
  I->size[0] = m;
  I->size[1] = (int)t;
  emxEnsureCapacity_real_T(I, i4);
  m *= (int)t;
  for (i4 = 0; i4 < m; i4++) {
    I->data[i4] = 0.0;
  }

  if (d > 0) {
    for (m = 0; m + 1 <= d; m++) {
      I->data[m + I->size[0] * m] = 1.0;
    }
  }
}

//
// Arguments    : double varargin_1
//                emxArray_real_T *I
// Return Type  : void
//
void eye(double varargin_1, emxArray_real_T *I)
{
  double t;
  int k;
  int loop_ub;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }

  k = I->size[0] * I->size[1];
  I->size[0] = (int)t;
  I->size[1] = (int)t;
  emxEnsureCapacity_real_T(I, k);
  loop_ub = (int)t * (int)t;
  for (k = 0; k < loop_ub; k++) {
    I->data[k] = 0.0;
  }

  if ((int)t > 0) {
    for (k = 0; k + 1 <= (int)t; k++) {
      I->data[k + I->size[0] * k] = 1.0;
    }
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
