/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xger.cpp
 *
 * Code generation for function 'xger'
 *
 */

/* Include files */
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/xger.h"

/* Function Definitions */
void xger(int m, int n, double alpha1, int ix0, const double y_data[], double
          A_data[], int ia0, int lda)
{
  int jA;
  int jy;
  int j;
  double temp;
  int ix;
  int i6;
  int ijA;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 1; j <= n; j++) {
      if (y_data[jy] != 0.0) {
        temp = y_data[jy] * alpha1;
        ix = ix0;
        i6 = m + jA;
        for (ijA = jA; ijA < i6; ijA++) {
          A_data[ijA] += A_data[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

/* End of code generation (xger.cpp) */
