/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.cpp
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "abs.h"
#include "solveQP_emxutil.h"

/* Function Definitions */
void b_abs(const emxArray_real_T *x, emxArray_real_T *y)
{
  int nx;
  unsigned int x_idx_0;
  int k;
  nx = x->size[0];
  x_idx_0 = (unsigned int)x->size[0];
  k = y->size[0];
  y->size[0] = (int)x_idx_0;
  emxEnsureCapacity_real_T(y, k);
  for (k = 0; k < nx; k++) {
    y->data[k] = std::abs(x->data[k]);
  }
}

/* End of code generation (abs.cpp) */
