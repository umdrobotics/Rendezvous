//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xger.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xger.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                double alpha1
//                int ix0
//                const emxArray_real_T *y
//                emxArray_real_T *A
//                int ia0
//                int lda
// Return Type  : void
//
void xger(int m, int n, double alpha1, int ix0, const emxArray_real_T *y,
          emxArray_real_T *A, int ia0, int lda)
{
  int jA;
  int jy;
  int j;
  double temp;
  int ix;
  int i14;
  int ijA;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 1; j <= n; j++) {
      if (y->data[jy] != 0.0) {
        temp = y->data[jy] * alpha1;
        ix = ix0;
        i14 = m + jA;
        for (ijA = jA; ijA + 1 <= i14; ijA++) {
          A->data[ijA] += A->data[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

//
// File trailer for xger.cpp
//
// [EOF]
//
