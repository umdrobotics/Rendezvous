//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//

// Include Files
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/xgemv.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int ia0
//                int lda
//                const double x_data[]
//                int ix0
//                double y_data[]
// Return Type  : void
//
void xgemv(int m, int n, const double A_data[], int ia0, int lda, const double
           x_data[], int ix0, double y_data[])
{
  int iy;
  int i4;
  int iac;
  int ix;
  double c;
  int i5;
  int ia;
  if (n != 0) {
    for (iy = 1; iy <= n; iy++) {
      y_data[iy - 1] = 0.0;
    }

    iy = 0;
    i4 = ia0 + lda * (n - 1);
    iac = ia0;
    while ((lda > 0) && (iac <= i4)) {
      ix = ix0;
      c = 0.0;
      i5 = (iac + m) - 1;
      for (ia = iac; ia <= i5; ia++) {
        c += A_data[ia - 1] * x_data[ix - 1];
        ix++;
      }

      y_data[iy] += c;
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
