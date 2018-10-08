//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qr.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
//

// Include Files
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/qr.h"
#include "navigation/xger.h"
#include "navigation/xgemv.h"
#include "navigation/xgeqrf.h"

// Function Definitions

//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                double Q_data[]
//                int Q_size[2]
//                double R_data[]
//                int R_size[2]
// Return Type  : void
//
void qr(const double A_data[], const int A_size[2], double Q_data[], int Q_size
        [2], double R_data[], int R_size[2])
{
  int m;
  int n;
  int b_A_size[2];
  int i;
  int b_i;
  double b_A_data[100];
  double tau_data[10];
  int tau_size[1];
  int itau;
  double work_data[10];
  int iaii;
  int c;
  int lastv;
  boolean_T exitg2;
  int ia;
  int exitg1;
  m = A_size[0];
  n = A_size[1];
  Q_size[0] = (signed char)A_size[0];
  Q_size[1] = (signed char)A_size[0];
  R_size[0] = A_size[0];
  R_size[1] = A_size[1];
  if (A_size[0] > A_size[1]) {
    for (i = 0; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        Q_data[b_i + Q_size[0] * i] = A_data[b_i + A_size[0] * i];
      }
    }

    for (i = A_size[1]; i + 1 <= m; i++) {
      for (b_i = 1; b_i <= m; b_i++) {
        Q_data[(b_i + Q_size[0] * i) - 1] = 0.0;
      }
    }

    xgeqrf(Q_data, Q_size, tau_data, tau_size);
    for (i = 0; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= i + 1; b_i++) {
        R_data[b_i + R_size[0] * i] = Q_data[b_i + Q_size[0] * i];
      }

      for (b_i = i + 1; b_i + 1 <= m; b_i++) {
        R_data[b_i + R_size[0] * i] = 0.0;
      }
    }

    if (!(A_size[0] < 1)) {
      for (i = A_size[1]; i < m; i++) {
        n = i * m;
        for (b_i = 0; b_i < m; b_i++) {
          Q_data[n + b_i] = 0.0;
        }

        Q_data[n + i] = 1.0;
      }

      itau = A_size[1] - 1;
      i = (signed char)Q_size[1];
      if (0 <= i - 1) {
        memset(&work_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
      }

      for (b_i = A_size[1]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          Q_data[iaii - 1] = 1.0;
          n = (m - b_i) - 1;
          c = iaii + m;
          if (tau_data[itau] != 0.0) {
            lastv = n + 2;
            i = iaii + n;
            while ((lastv > 0) && (Q_data[i] == 0.0)) {
              lastv--;
              i--;
            }

            n = m - b_i;
            exitg2 = false;
            while ((!exitg2) && (n > 0)) {
              i = c + (n - 1) * m;
              ia = i;
              do {
                exitg1 = 0;
                if (ia <= (i + lastv) - 1) {
                  if (Q_data[ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    ia++;
                  }
                } else {
                  n--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }
          } else {
            lastv = 0;
            n = 0;
          }

          if (lastv > 0) {
            xgemv(lastv, n, Q_data, c, m, Q_data, iaii, work_data);
            xger(lastv, n, -tau_data[itau], iaii, work_data, Q_data, c, m);
          }

          i = (iaii + m) - b_i;
          for (n = iaii; n + 1 <= i; n++) {
            Q_data[n] *= -tau_data[itau];
          }
        }

        Q_data[iaii - 1] = 1.0 - tau_data[itau];
        for (i = 1; i < b_i; i++) {
          Q_data[(iaii - i) - 1] = 0.0;
        }

        itau--;
      }
    }
  } else {
    b_A_size[0] = A_size[0];
    b_A_size[1] = A_size[1];
    i = A_size[0] * A_size[1];
    if (0 <= i - 1) {
      memcpy(&b_A_data[0], &A_data[0], (unsigned int)(i * (int)sizeof(double)));
    }

    xgeqrf(b_A_data, b_A_size, tau_data, tau_size);
    for (i = 0; i + 1 <= m; i++) {
      for (b_i = 0; b_i + 1 <= i + 1; b_i++) {
        R_data[b_i + R_size[0] * i] = b_A_data[b_i + b_A_size[0] * i];
      }

      for (b_i = i + 1; b_i + 1 <= m; b_i++) {
        R_data[b_i + R_size[0] * i] = 0.0;
      }
    }

    for (i = A_size[0]; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        R_data[b_i + R_size[0] * i] = b_A_data[b_i + b_A_size[0] * i];
      }
    }

    if (!(A_size[0] < 1)) {
      for (i = A_size[0]; i < m; i++) {
        n = i * m;
        for (b_i = 0; b_i < m; b_i++) {
          b_A_data[n + b_i] = 0.0;
        }

        b_A_data[n + i] = 1.0;
      }

      itau = A_size[0] - 1;
      i = (signed char)b_A_size[1];
      if (0 <= i - 1) {
        memset(&work_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
      }

      for (b_i = A_size[0]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          b_A_data[iaii - 1] = 1.0;
          n = (m - b_i) - 1;
          c = iaii + m;
          if (tau_data[itau] != 0.0) {
            lastv = n + 2;
            i = iaii + n;
            while ((lastv > 0) && (b_A_data[i] == 0.0)) {
              lastv--;
              i--;
            }

            n = m - b_i;
            exitg2 = false;
            while ((!exitg2) && (n > 0)) {
              i = c + (n - 1) * m;
              ia = i;
              do {
                exitg1 = 0;
                if (ia <= (i + lastv) - 1) {
                  if (b_A_data[ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    ia++;
                  }
                } else {
                  n--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);

              if (exitg1 == 1) {
                exitg2 = true;
              }
            }
          } else {
            lastv = 0;
            n = 0;
          }

          if (lastv > 0) {
            xgemv(lastv, n, b_A_data, c, m, b_A_data, iaii, work_data);
            xger(lastv, n, -tau_data[itau], iaii, work_data, b_A_data, c, m);
          }

          i = (iaii + m) - b_i;
          for (n = iaii; n + 1 <= i; n++) {
            b_A_data[n] *= -tau_data[itau];
          }
        }

        b_A_data[iaii - 1] = 1.0 - tau_data[itau];
        for (i = 1; i < b_i; i++) {
          b_A_data[(iaii - i) - 1] = 0.0;
        }

        itau--;
      }
    }

    for (i = 0; i + 1 <= m; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        Q_data[b_i + Q_size[0] * i] = b_A_data[b_i + b_A_size[0] * i];
      }
    }
  }
}

//
// File trailer for qr.cpp
//
// [EOF]
//
