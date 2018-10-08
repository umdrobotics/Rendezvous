//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
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
  int i8;
  int iac;
  int ix;
  double c;
  int i9;
  int ia;
  if (n != 0) {
    for (iy = 1; iy <= n; iy++) {
      y_data[iy - 1] = 0.0;
    }

    iy = 0;
    i8 = ia0 + lda * (n - 1);
    iac = ia0;
    while ((lda > 0) && (iac <= i8)) {
      ix = ix0;
      c = 0.0;
      i9 = (iac + m) - 1;
      for (ia = iac; ia <= i9; ia++) {
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
