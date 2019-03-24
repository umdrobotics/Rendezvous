//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "norm.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
// Return Type  : double
//
double norm(const emxArray_real_T *x)
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  if (x->size[0] == 1) {
    y = std::abs(x->data[0]);
  } else {
    scale = 3.3121686421112381E-170;
    for (k = 1; k <= x->size[0]; k++) {
      absxk = std::abs(x->data[k - 1]);
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

//
// File trailer for norm.cpp
//
// [EOF]
//
