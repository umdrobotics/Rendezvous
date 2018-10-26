/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qr.cpp
 *
 * Code generation for function 'qr'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "qr.h"
#include "xscal.h"
#include "xger.h"
#include "xgemv.h"
#include "solveQP_emxutil.h"
#include "xgeqrf.h"

/* Function Definitions */
void qr(const emxArray_real_T *A, emxArray_real_T *Q, emxArray_real_T *R)
{
  int m;
  int n;
  int coltop;
  int i;
  int lastv;
  emxArray_real_T *tau;
  emxArray_real_T *work;
  emxArray_real_T *b_A;
  int b_i;
  int itau;
  int ia;
  int iaii;
  int lastc;
  boolean_T exitg2;
  int exitg1;
  m = A->size[0];
  n = A->size[1];
  coltop = A->size[0];
  i = A->size[0];
  lastv = Q->size[0] * Q->size[1];
  Q->size[0] = coltop;
  Q->size[1] = i;
  emxEnsureCapacity_real_T(Q, lastv);
  lastv = R->size[0] * R->size[1];
  R->size[0] = A->size[0];
  R->size[1] = A->size[1];
  emxEnsureCapacity_real_T(R, lastv);
  emxInit_real_T(&tau, 1);
  emxInit_real_T(&work, 1);
  if (A->size[0] > A->size[1]) {
    for (coltop = 0; coltop < n; coltop++) {
      for (b_i = 0; b_i < m; b_i++) {
        Q->data[b_i + Q->size[0] * coltop] = A->data[b_i + A->size[0] * coltop];
      }
    }

    lastv = A->size[1] + 1;
    for (coltop = lastv; coltop <= m; coltop++) {
      for (b_i = 0; b_i < m; b_i++) {
        Q->data[b_i + Q->size[0] * (coltop - 1)] = 0.0;
      }
    }

    xgeqrf(Q, tau);
    for (coltop = 0; coltop < n; coltop++) {
      for (b_i = 0; b_i <= coltop; b_i++) {
        R->data[b_i + R->size[0] * coltop] = Q->data[b_i + Q->size[0] * coltop];
      }

      lastv = coltop + 2;
      for (b_i = lastv; b_i <= m; b_i++) {
        R->data[(b_i + R->size[0] * coltop) - 1] = 0.0;
      }
    }

    if (A->size[0] >= 1) {
      lastv = A->size[0] - 1;
      for (coltop = n; coltop <= lastv; coltop++) {
        ia = coltop * m;
        i = m - 1;
        for (b_i = 0; b_i <= i; b_i++) {
          Q->data[ia + b_i] = 0.0;
        }

        Q->data[ia + coltop] = 1.0;
      }

      itau = A->size[1] - 1;
      coltop = Q->size[1];
      lastv = work->size[0];
      work->size[0] = coltop;
      emxEnsureCapacity_real_T(work, lastv);
      for (lastv = 0; lastv < coltop; lastv++) {
        work->data[lastv] = 0.0;
      }

      for (b_i = A->size[1]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          Q->data[iaii - 1] = 1.0;
          n = iaii + m;
          if (tau->data[itau] != 0.0) {
            lastc = m - b_i;
            lastv = lastc + 1;
            i = n - b_i;
            while ((lastv > 0) && (Q->data[i - 1] == 0.0)) {
              lastv--;
              i--;
            }

            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
              coltop = n + (lastc - 1) * m;
              ia = coltop;
              do {
                exitg1 = 0;
                if (ia <= (coltop + lastv) - 1) {
                  if (Q->data[ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    ia++;
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
        for (coltop = 0; coltop <= b_i - 2; coltop++) {
          Q->data[(iaii - coltop) - 2] = 0.0;
        }

        itau--;
      }
    }
  } else {
    emxInit_real_T(&b_A, 2);
    lastv = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, lastv);
    i = A->size[0] * A->size[1];
    for (lastv = 0; lastv < i; lastv++) {
      b_A->data[lastv] = A->data[lastv];
    }

    xgeqrf(b_A, tau);
    for (coltop = 0; coltop < m; coltop++) {
      for (b_i = 0; b_i <= coltop; b_i++) {
        R->data[b_i + R->size[0] * coltop] = b_A->data[b_i + b_A->size[0] *
          coltop];
      }

      lastv = coltop + 2;
      for (b_i = lastv; b_i <= m; b_i++) {
        R->data[(b_i + R->size[0] * coltop) - 1] = 0.0;
      }
    }

    lastv = A->size[0] + 1;
    for (coltop = lastv; coltop <= n; coltop++) {
      for (b_i = 0; b_i < m; b_i++) {
        R->data[b_i + R->size[0] * (coltop - 1)] = b_A->data[b_i + b_A->size[0] *
          (coltop - 1)];
      }
    }

    if (A->size[0] >= 1) {
      lastv = A->size[0] - 1;
      for (coltop = m; coltop <= lastv; coltop++) {
        ia = coltop * m;
        i = m - 1;
        for (b_i = 0; b_i <= i; b_i++) {
          b_A->data[ia + b_i] = 0.0;
        }

        b_A->data[ia + coltop] = 1.0;
      }

      itau = A->size[0] - 1;
      coltop = b_A->size[1];
      lastv = work->size[0];
      work->size[0] = coltop;
      emxEnsureCapacity_real_T(work, lastv);
      for (lastv = 0; lastv < coltop; lastv++) {
        work->data[lastv] = 0.0;
      }

      for (b_i = A->size[0]; b_i >= 1; b_i--) {
        iaii = b_i + (b_i - 1) * m;
        if (b_i < m) {
          b_A->data[iaii - 1] = 1.0;
          n = iaii + m;
          if (tau->data[itau] != 0.0) {
            lastc = m - b_i;
            lastv = lastc + 1;
            i = n - b_i;
            while ((lastv > 0) && (b_A->data[i - 1] == 0.0)) {
              lastv--;
              i--;
            }

            exitg2 = false;
            while ((!exitg2) && (lastc > 0)) {
              coltop = n + (lastc - 1) * m;
              ia = coltop;
              do {
                exitg1 = 0;
                if (ia <= (coltop + lastv) - 1) {
                  if (b_A->data[ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    ia++;
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
        for (coltop = 0; coltop <= b_i - 2; coltop++) {
          b_A->data[(iaii - coltop) - 2] = 0.0;
        }

        itau--;
      }
    }

    for (coltop = 0; coltop < m; coltop++) {
      for (b_i = 0; b_i < m; b_i++) {
        Q->data[b_i + Q->size[0] * coltop] = b_A->data[b_i + b_A->size[0] *
          coltop];
      }
    }

    emxFree_real_T(&b_A);
  }

  emxFree_real_T(&work);
  emxFree_real_T(&tau);
}

/* End of code generation (qr.cpp) */
