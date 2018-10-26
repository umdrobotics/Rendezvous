/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgeqrf.cpp
 *
 * Code generation for function 'xgeqrf'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xgeqrf.h"
#include "solveQP_emxutil.h"
#include "xger.h"
#include "xgemv.h"
#include "xscal.h"
#include "xnrm2.h"

/* Function Declarations */
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
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

void xgeqrf(emxArray_real_T *A, emxArray_real_T *tau)
{
  int m;
  int n;
  int lastc;
  int mn;
  emxArray_real_T *work;
  int knt;
  int i;
  int i_i;
  int lastv;
  int mmip1;
  double atmp;
  double xnorm;
  int ic0;
  double beta1;
  boolean_T exitg2;
  int exitg1;
  m = A->size[0];
  n = A->size[1];
  lastc = A->size[0];
  mn = A->size[1];
  if (lastc < mn) {
    mn = lastc;
  }

  lastc = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity_real_T(tau, lastc);
  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    emxInit_real_T(&work, 1);
    knt = A->size[1];
    lastc = work->size[0];
    work->size[0] = knt;
    emxEnsureCapacity_real_T(work, lastc);
    for (lastc = 0; lastc < knt; lastc++) {
      work->data[lastc] = 0.0;
    }

    for (i = 0; i < mn; i++) {
      i_i = i + i * m;
      lastv = m - i;
      mmip1 = lastv - 1;
      if (i + 1 < m) {
        atmp = A->data[i_i];
        tau->data[i] = 0.0;
        if (lastv > 0) {
          xnorm = xnrm2(mmip1, A, i_i + 2);
          if (xnorm != 0.0) {
            beta1 = rt_hypotd_snf(A->data[i_i], xnorm);
            if (A->data[i_i] >= 0.0) {
              beta1 = -beta1;
            }

            if (std::abs(beta1) < 1.0020841800044864E-292) {
              knt = -1;
              do {
                knt++;
                xscal(mmip1, 9.9792015476736E+291, A, i_i + 2);
                beta1 *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

              beta1 = rt_hypotd_snf(atmp, xnrm2(lastv - 1, A, i_i + 2));
              if (atmp >= 0.0) {
                beta1 = -beta1;
              }

              tau->data[i] = (beta1 - atmp) / beta1;
              xscal(mmip1, 1.0 / (atmp - beta1), A, i_i + 2);
              for (lastc = 0; lastc <= knt; lastc++) {
                beta1 *= 1.0020841800044864E-292;
              }

              atmp = beta1;
            } else {
              tau->data[i] = (beta1 - A->data[i_i]) / beta1;
              xnorm = 1.0 / (A->data[i_i] - beta1);
              xscal(mmip1, xnorm, A, i_i + 2);
              atmp = beta1;
            }
          }
        }

        A->data[i_i] = atmp;
      } else {
        tau->data[i] = 0.0;
      }

      if (i + 1 < n) {
        atmp = A->data[i_i];
        A->data[i_i] = 1.0;
        ic0 = (i + (i + 1) * m) + 1;
        if (tau->data[i] != 0.0) {
          lastc = i_i + mmip1;
          while ((lastv > 0) && (A->data[lastc] == 0.0)) {
            lastv--;
            lastc--;
          }

          lastc = (n - i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            knt = ic0 + (lastc - 1) * m;
            mmip1 = knt;
            do {
              exitg1 = 0;
              if (mmip1 <= (knt + lastv) - 1) {
                if (A->data[mmip1 - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  mmip1++;
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
          xgemv(lastv, lastc, A, ic0, m, A, i_i + 1, work);
          xger(lastv, lastc, -tau->data[i], i_i + 1, work, A, ic0, m);
        }

        A->data[i_i] = atmp;
      }
    }

    emxFree_real_T(&work);
  }
}

/* End of code generation (xgeqrf.cpp) */
