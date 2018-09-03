/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xpotrf.cpp
 *
 * Code generation for function 'xpotrf'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/xpotrf.h"

/* Function Definitions */
int xpotrf(double A_data[], int A_size[2])
{
  int info;
  int j;
  boolean_T exitg1;
  int jj;
  double ajj;
  int ix;
  int iy;
  int k;
  int i4;
  double x_data[400];
  double c;
  int i5;
  int ia;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j + 1 <= 20)) {
    jj = j + j * 20;
    ajj = 0.0;
    if (!(j < 1)) {
      ix = j;
      iy = j;
      for (k = 1; k <= j; k++) {
        ajj += A_data[ix] * A_data[iy];
        ix += 20;
        iy += 20;
      }
    }

    ajj = A_data[jj] - ajj;
    if (ajj > 0.0) {
      ajj = std::sqrt(ajj);
      A_data[jj] = ajj;
      if (j + 1 < 20) {
        if (j != 0) {
          ix = j;
          i4 = (j + 20 * (j - 1)) + 2;
          for (k = j + 2; k <= i4; k += 20) {
            c = -A_data[ix];
            iy = jj + 1;
            i5 = (k - j) + 18;
            for (ia = k; ia <= i5; ia++) {
              A_data[iy] += A_data[ia - 1] * c;
              iy++;
            }

            ix += 20;
          }
        }

        ajj = 1.0 / ajj;
        memcpy(&x_data[0], &A_data[0], 400U * sizeof(double));
        i4 = (jj - j) + 20;
        for (k = jj + 1; k < i4; k++) {
          x_data[k] *= ajj;
        }

        A_size[0] = 20;
        A_size[1] = 20;
        for (i4 = 0; i4 < 20; i4++) {
          for (i5 = 0; i5 < 20; i5++) {
            A_data[i5 + A_size[0] * i4] = x_data[i5 + 20 * i4];
          }
        }
      }

      j++;
    } else {
      A_data[jj] = ajj;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* End of code generation (xpotrf.cpp) */
