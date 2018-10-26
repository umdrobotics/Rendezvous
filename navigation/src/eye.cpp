/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eye.cpp
 *
 * Code generation for function 'eye'
 *
 */

/* Include files */
#include <string.h>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "eye.h"
#include "solveQP_emxutil.h"

/* Function Definitions */
void b_eye(double b_I[16])
{
  memset(&b_I[0], 0, sizeof(double) << 4);
  b_I[0] = 1.0;
  b_I[5] = 1.0;
  b_I[10] = 1.0;
  b_I[15] = 1.0;
}

void c_eye(double varargin_1, double varargin_2, emxArray_real_T *b_I)
{
  int m;
  double t;
  int d;
  int i6;
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

  i6 = b_I->size[0] * b_I->size[1];
  b_I->size[0] = m;
  b_I->size[1] = (int)t;
  emxEnsureCapacity_real_T(b_I, i6);
  m *= (int)t;
  for (i6 = 0; i6 < m; i6++) {
    b_I->data[i6] = 0.0;
  }

  if (d > 0) {
    for (m = 0; m < d; m++) {
      b_I->data[m + b_I->size[0] * m] = 1.0;
    }
  }
}

void eye(double varargin_1, emxArray_real_T *b_I)
{
  double t;
  int m;
  int k;
  int loop_ub;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }

  m = (int)t;
  k = b_I->size[0] * b_I->size[1];
  b_I->size[0] = (int)t;
  b_I->size[1] = (int)t;
  emxEnsureCapacity_real_T(b_I, k);
  loop_ub = (int)t * (int)t;
  for (k = 0; k < loop_ub; k++) {
    b_I->data[k] = 0.0;
  }

  if ((int)t > 0) {
    for (k = 0; k < m; k++) {
      b_I->data[k + b_I->size[0] * k] = 1.0;
    }
  }
}

/* End of code generation (eye.cpp) */
