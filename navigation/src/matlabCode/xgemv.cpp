//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xgemv.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int ia0
//                int lda
//                const emxArray_real_T *x
//                int ix0
//                emxArray_real_T *y
// Return Type  : void
//
void xgemv(int m, int n, const emxArray_real_T *A, int ia0, int lda, const
           emxArray_real_T *x, int ix0, emxArray_real_T *y)
{
  int iy;
  int i12;
  int iac;
  int ix;
  double c;
  int i13;
  int ia;
  if (n != 0) {
    for (iy = 1; iy <= n; iy++) {
      y->data[iy - 1] = 0.0;
    }

    iy = 0;
    i12 = ia0 + lda * (n - 1);
    iac = ia0;
    while ((lda > 0) && (iac <= i12)) {
      ix = ix0;
      c = 0.0;
      i13 = (iac + m) - 1;
      for (ia = iac; ia <= i13; ia++) {
        c += A->data[ia - 1] * x->data[ix - 1];
        ix++;
      }

      y->data[iy] += c;
      iy++;
      iac += lda;
    }
  }
}

//
// File trailer for xgemv.cpp
//
// [EOF]
//
