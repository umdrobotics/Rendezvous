//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//

// Include Files
#include <cmath>
#include <string.h>
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/inv.h"

// Function Declarations
static void invNxN(const double x_data[], const int x_size[2], double y_data[],
                   int y_size[2]);

// Function Definitions

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void invNxN(const double x_data[], const int x_size[2], double y_data[],
                   int y_size[2])
{
  int n;
  int yk;
  double b_x_data[100];
  int b_n;
  int ipiv_data[10];
  int jA;
  int k;
  int u1;
  int p_data[10];
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int i3;
  int jy;
  double s;
  int ijA;
  n = x_size[0];
  y_size[0] = x_size[0];
  y_size[1] = x_size[1];
  yk = x_size[0] * x_size[1];
  if (0 <= yk - 1) {
    memset(&y_data[0], 0, (unsigned int)(yk * (int)sizeof(double)));
  }

  yk = x_size[0] * x_size[1];
  if (0 <= yk - 1) {
    memcpy(&b_x_data[0], &x_data[0], (unsigned int)(yk * (int)sizeof(double)));
  }

  if (x_size[0] < 1) {
    b_n = 0;
  } else {
    b_n = x_size[0];
  }

  if (b_n > 0) {
    ipiv_data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv_data[k - 1] = yk;
    }
  }

  if (x_size[0] < 1) {
    jA = 0;
  } else {
    yk = x_size[0] - 1;
    u1 = x_size[0];
    if (yk < u1) {
      u1 = yk;
    }

    for (j = 0; j < u1; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          ix = c;
          smax = std::abs(b_x_data[c]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = std::abs(b_x_data[ix]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (b_x_data[c + yk] != 0.0) {
        if (yk != 0) {
          ipiv_data[j] = (j + yk) + 1;
          ix = j;
          yk += j;
          for (k = 1; k <= n; k++) {
            smax = b_x_data[ix];
            b_x_data[ix] = b_x_data[yk];
            b_x_data[yk] = smax;
            ix += n;
            yk += n;
          }
        }

        i3 = c + mmj;
        for (jy = c + 1; jy < i3; jy++) {
          b_x_data[jy] /= b_x_data[c];
        }
      }

      yk = n - j;
      jA = (c + n) + 1;
      jy = c + n;
      for (k = 1; k < yk; k++) {
        smax = b_x_data[jy];
        if (b_x_data[jy] != 0.0) {
          ix = c + 1;
          i3 = mmj + jA;
          for (ijA = jA; ijA < i3 - 1; ijA++) {
            b_x_data[ijA] += b_x_data[ix] * -smax;
            ix++;
          }
        }

        jy += n;
        jA += n;
      }
    }

    jA = x_size[0];
  }

  if (jA > 0) {
    p_data[0] = 1;
    yk = 1;
    for (k = 2; k <= jA; k++) {
      yk++;
      p_data[k - 1] = yk;
    }
  }

  for (k = 0; k < b_n; k++) {
    if (ipiv_data[k] > 1 + k) {
      yk = p_data[ipiv_data[k] - 1];
      p_data[ipiv_data[k] - 1] = p_data[k];
      p_data[k] = yk;
    }
  }

  for (k = 0; k < n; k++) {
    c = p_data[k] - 1;
    y_data[k + y_size[0] * (p_data[k] - 1)] = 1.0;
    for (j = k; j < n; j++) {
      if (y_data[j + y_size[0] * c] != 0.0) {
        for (jy = j + 1; jy < n; jy++) {
          y_data[jy + y_size[0] * c] -= y_data[j + y_size[0] * c] * b_x_data[jy
            + x_size[0] * j];
        }
      }
    }
  }

  if ((x_size[0] == 0) || ((x_size[0] == 0) || (x_size[1] == 0))) {
  } else {
    for (j = 1; j <= n; j++) {
      yk = n * (j - 1) - 1;
      for (k = n; k > 0; k--) {
        jA = n * (k - 1) - 1;
        if (y_data[k + yk] != 0.0) {
          y_data[k + yk] /= b_x_data[k + jA];
          for (jy = 1; jy < k; jy++) {
            y_data[jy + yk] -= y_data[k + yk] * b_x_data[jy + jA];
          }
        }
      }
    }
  }
}

//
// Arguments    : double x_data[]
//                int x_size[2]
// Return Type  : void
//
void inv(double x_data[], int x_size[2])
{
  int b_x_size[2];
  int loop_ub;
  double b_x_data[100];
  if (!((x_size[0] == 0) || (x_size[1] == 0))) {
    b_x_size[0] = x_size[0];
    b_x_size[1] = x_size[1];
    loop_ub = x_size[0] * x_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&b_x_data[0], &x_data[0], (unsigned int)(loop_ub * (int)sizeof
              (double)));
    }

    invNxN(b_x_data, b_x_size, x_data, x_size);
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
