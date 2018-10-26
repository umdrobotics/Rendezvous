/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * norm.cpp
 *
 * Code generation for function 'norm'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "norm.h"

/* Function Definitions */
double b_norm(const emxArray_real_T *x)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (x->size[0] == 1) {
    y = std::abs(x->data[0]);
  } else {
    scale = 3.3121686421112381E-170;
    kend = x->size[0];
    for (k = 0; k < kend; k++) {
      absxk = std::abs(x->data[k]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0 + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * std::sqrt(y);
  }

  return y;
}

/* End of code generation (norm.cpp) */
