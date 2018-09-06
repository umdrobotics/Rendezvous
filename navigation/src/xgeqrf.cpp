//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeqrf.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//

// Include Files
#include <cmath>
#include "navigation/rt_nonfinite.h"
#include <string.h>
#include "navigation/solveQP.h"
#include "navigation/xgeqrf.h"
#include "navigation/xger.h"
#include "navigation/xgemv.h"
#include "navigation/xnrm2.h"

// Function Declarations
static double rt_hypotd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = b * std::sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * std::sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = b;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

//
// Arguments    : double A_data[]
//                int A_size[2]
//                double tau_data[]
//                int tau_size[1]
// Return Type  : void
//
void xgeqrf(double A_data[], int A_size[2], double tau_data[], int tau_size[1])
{
  int m;
  int n;
  int lastc;
  int mn;
  double work_data[10];
  int i;
  int i_i;
  int mmi;
  double atmp;
  double d1;
  double xnorm;
  int i_ip1;
  int lastv;
  int knt;
  boolean_T exitg2;
  int coltop;
  int exitg1;
  m = A_size[0];
  n = A_size[1];
  lastc = A_size[0];
  mn = A_size[1];
  if (lastc < mn) {
    mn = lastc;
  }

  tau_size[0] = (signed char)mn;
  if (!((A_size[0] == 0) || (A_size[1] == 0))) {
    lastc = (signed char)A_size[1];
    if (0 <= lastc - 1) {
      memset(&work_data[0], 0, (unsigned int)(lastc * (int)sizeof(double)));
    }

    for (i = 1; i <= mn; i++) {
      i_i = (i + (i - 1) * m) - 1;
      mmi = m - i;
      if (i < m) {
        atmp = A_data[i_i];
        d1 = 0.0;
        if (!(1 + mmi <= 0)) {
          xnorm = xnrm2(mmi, A_data, i_i + 2);
          if (xnorm != 0.0) {
            xnorm = rt_hypotd_snf(A_data[i_i], xnorm);
            if (A_data[i_i] >= 0.0) {
              xnorm = -xnorm;
            }

            if (std::abs(xnorm) < 1.0020841800044864E-292) {
              knt = 0;
              lastc = i_i + mmi;
              do {
                knt++;
                for (coltop = i_i + 1; coltop < lastc + 1; coltop++) {
                  A_data[coltop] *= 9.9792015476736E+291;
                }

                xnorm *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while (!(std::abs(xnorm) >= 1.0020841800044864E-292));

              xnorm = rt_hypotd_snf(atmp, xnrm2(mmi, A_data, i_i + 2));
              if (atmp >= 0.0) {
                xnorm = -xnorm;
              }

              d1 = (xnorm - atmp) / xnorm;
              atmp = 1.0 / (atmp - xnorm);
              lastc = i_i + mmi;
              for (coltop = i_i + 1; coltop < lastc + 1; coltop++) {
                A_data[coltop] *= atmp;
              }

              for (coltop = 1; coltop <= knt; coltop++) {
                xnorm *= 1.0020841800044864E-292;
              }

              atmp = xnorm;
            } else {
              d1 = (xnorm - A_data[i_i]) / xnorm;
              atmp = 1.0 / (A_data[i_i] - xnorm);
              lastc = i_i + mmi;
              for (coltop = i_i + 1; coltop < lastc + 1; coltop++) {
                A_data[coltop] *= atmp;
              }

              atmp = xnorm;
            }
          }
        }

        tau_data[i - 1] = d1;
        A_data[i_i] = atmp;
      } else {
        tau_data[i - 1] = 0.0;
      }

      if (i < n) {
        atmp = A_data[i_i];
        A_data[i_i] = 1.0;
        i_ip1 = i + i * m;
        if (tau_data[i - 1] != 0.0) {
          lastv = 1 + mmi;
          lastc = i_i + mmi;
          while ((lastv > 0) && (A_data[lastc] == 0.0)) {
            lastv--;
            lastc--;
          }

          lastc = n - i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            coltop = i_ip1 + (lastc - 1) * m;
            knt = coltop;
            do {
              exitg1 = 0;
              if (knt <= (coltop + lastv) - 1) {
                if (A_data[knt - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  knt++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }

        if (lastv > 0) {
          xgemv(lastv, lastc, A_data, i_ip1, m, A_data, i_i + 1, work_data);
          xger(lastv, lastc, -tau_data[i - 1], i_i + 1, work_data, A_data, i_ip1,
               m);
        }

        A_data[i_i] = atmp;
      }
    }
  }
}

//
// File trailer for xgeqrf.cpp
//
// [EOF]
//
