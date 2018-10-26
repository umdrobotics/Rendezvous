/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgemv.cpp
 *
 * Code generation for function 'xgemv'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xgemv.h"

/* Function Definitions */
void xgemv(int m, int n, const emxArray_real_T *A, int ia0, int lda, const
           emxArray_real_T *x, int ix0, emxArray_real_T *y)
{
  int iy;
  int i18;
  int iac;
  int ix;
  double c;
  int i19;
  int ia;
  if (n != 0) {
    for (iy = 0; iy < n; iy++) {
      y->data[iy] = 0.0;
    }

    iy = 0;
    i18 = ia0 + lda * (n - 1);
    for (iac = ia0; lda < 0 ? iac >= i18 : iac <= i18; iac += lda) {
      ix = ix0;
      c = 0.0;
      i19 = (iac + m) - 1;
      for (ia = iac; ia <= i19; ia++) {
        c += A->data[ia - 1] * x->data[ix - 1];
        ix++;
      }

      y->data[iy] += c;
      iy++;
    }
  }
}

/* End of code generation (xgemv.cpp) */
