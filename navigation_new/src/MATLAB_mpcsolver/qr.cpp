//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qr.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "qr.h"
#include "xscal.h"
#include "xger.h"
#include "xgemv.h"
#include "solveQP_emxutil.h"
#include "xgeqrf.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *Q
//                emxArray_real_T *R
// Return Type  : void
//
void qr(const emxArray_real_T *A, emxArray_real_T *Q, emxArray_real_T *R)
{
  int m;
  int n;
  int lastc;
  int i;
  int coltop;
  emxArray_real_T *tau;
  emxArray_real_T *work;
  emxArray_real_T *b_A;
  int b_i;
  int itau;
  int iaii;
  int lastv;
  boolean_T exitg2;
  int exitg1;
  m = A->size[0];
  n = A->size[1];
  lastc = A->size[0];
  i = A->size[0];
  coltop = Q->size[0] * Q->size[1];
  Q->size[0] = lastc;
  Q->size[1] = i;
  emxEnsureCapacity_real_T(Q, coltop);
  coltop = R->size[0] * R->size[1];
  R->size[0] = A->size[0];
  R->size[1] = A->size[1];
  emxEnsureCapacity_real_T(R, coltop);
  emxInit_real_T1(&tau, 1);
  emxInit_real_T1(&work, 1);
  if (A->size[0] > A->size[1]) {
    for (i = 0; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        Q->data[b_i + Q->size[0] * i] = A->data[b_i + A->size[0] * i];
      }
    }

    for (i = A->size[1]; i + 1 <= m; i++) {
      for (b_i = 1; b_i <= m; b_i++) {
        Q->data[(b_i + Q->size[0] * i) - 1] = 0.0;
      }
    }

    xgeqrf(Q, tau);
    for (i = 0; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= i + 1; b_i++) {
        R->data[b_i + R->size[0] * i] = Q->data[b_i + Q->size[0] * i];
      }

      for (b_i = i + 1; b_i + 1 <= m; b_i++) {
        R->data[b_i + R->size[0] * i] = 0.0;
      }
    }

    if (!(A->size[0] < 1)) {
      for (i = A->size[1]; i < m; i++) {
        lastc = i * m;
        for (b_i = 0; b_i < m; b_i++) {
          Q->data[lastc + b_i] = 0.0;
        }

        Q->data[lastc + i] = 1.0;
      }

      itau = A->size[1] - 1;
      lastc = Q->size[1];
      coltop = work->size[0];
      work->size[0] = lastc;
      emxEnsureCapacity_real_T1(work, coltop);
      for (coltop = 0; coltop < lastc; coltop++) {
        work->data[coltop] = 0.0;
      }

      for (b_i = A->size[1]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          Q->data[iaii - 1] = 1.0;
          lastc = (m - b_i) - 1;
          n = iaii + m;
          if (tau->data[itau] != 0.0) {
            lastv = lastc + 2;
            i = iaii + lastc;
            while ((lastv > 0) && (Q->data[i] == 0.0)) {
              lastv--;
              i--;
            }

            lastc = m - b_i;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
              coltop = n + (lastc - 1) * m;
              i = coltop;
              do {
                exitg1 = 0;
                if (i <= (coltop + lastv) - 1) {
                  if (Q->data[i - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    i++;
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
            xgemv(lastv, lastc, Q, n, m, Q, iaii, work);
            xger(lastv, lastc, -tau->data[itau], iaii, work, Q, n, m);
          }

          xscal(m - b_i, -tau->data[itau], Q, iaii + 1);
        }

        Q->data[iaii - 1] = 1.0 - tau->data[itau];
        for (i = 1; i < b_i; i++) {
          Q->data[(iaii - i) - 1] = 0.0;
        }

        itau--;
      }
    }
  } else {
    emxInit_real_T(&b_A, 2);
    coltop = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, coltop);
    i = A->size[0] * A->size[1];
    for (coltop = 0; coltop < i; coltop++) {
      b_A->data[coltop] = A->data[coltop];
    }

    xgeqrf(b_A, tau);
    for (i = 0; i + 1 <= m; i++) {
      for (b_i = 0; b_i + 1 <= i + 1; b_i++) {
        R->data[b_i + R->size[0] * i] = b_A->data[b_i + b_A->size[0] * i];
      }

      for (b_i = i + 1; b_i + 1 <= m; b_i++) {
        R->data[b_i + R->size[0] * i] = 0.0;
      }
    }

    for (i = A->size[0]; i + 1 <= n; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        R->data[b_i + R->size[0] * i] = b_A->data[b_i + b_A->size[0] * i];
      }
    }

    if (!(A->size[0] < 1)) {
      for (i = A->size[0]; i < m; i++) {
        lastc = i * m;
        for (b_i = 0; b_i < m; b_i++) {
          b_A->data[lastc + b_i] = 0.0;
        }

        b_A->data[lastc + i] = 1.0;
      }

      itau = A->size[0] - 1;
      lastc = b_A->size[1];
      coltop = work->size[0];
      work->size[0] = lastc;
      emxEnsureCapacity_real_T1(work, coltop);
      for (coltop = 0; coltop < lastc; coltop++) {
        work->data[coltop] = 0.0;
      }

      for (b_i = A->size[0]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          b_A->data[iaii - 1] = 1.0;
          lastc = (m - b_i) - 1;
          n = iaii + m;
          if (tau->data[itau] != 0.0) {
            lastv = lastc + 2;
            i = iaii + lastc;
            while ((lastv > 0) && (b_A->data[i] == 0.0)) {
              lastv--;
              i--;
            }

            lastc = m - b_i;
            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
              coltop = n + (lastc - 1) * m;
              i = coltop;
              do {
                exitg1 = 0;
                if (i <= (coltop + lastv) - 1) {
                  if (b_A->data[i - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    i++;
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
            xgemv(lastv, lastc, b_A, n, m, b_A, iaii, work);
            xger(lastv, lastc, -tau->data[itau], iaii, work, b_A, n, m);
          }

          xscal(m - b_i, -tau->data[itau], b_A, iaii + 1);
        }

        b_A->data[iaii - 1] = 1.0 - tau->data[itau];
        for (i = 1; i < b_i; i++) {
          b_A->data[(iaii - i) - 1] = 0.0;
        }

        itau--;
      }
    }

    for (i = 0; i + 1 <= m; i++) {
      for (b_i = 0; b_i + 1 <= m; b_i++) {
        Q->data[b_i + Q->size[0] * i] = b_A->data[b_i + b_A->size[0] * i];
      }
    }

    emxFree_real_T(&b_A);
  }

  emxFree_real_T(&work);
  emxFree_real_T(&tau);
}

//
// File trailer for qr.cpp
//
// [EOF]
//
