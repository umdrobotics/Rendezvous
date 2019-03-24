//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeqrf.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xgeqrf.h"
#include "xger.h"
#include "xgemv.h"
#include "xscal.h"
#include "xnrm2.h"
#include "solveQP_emxutil.h"

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
// Arguments    : emxArray_real_T *A
//                emxArray_real_T *tau
// Return Type  : void
//
void xgeqrf(emxArray_real_T *A, emxArray_real_T *tau)
{
  int m;
  int n;
  int knt;
  int mn;
  emxArray_real_T *work;
  int i;
  int b_i;
  int i_i;
  int mmi;
  double atmp;
  double d0;
  double xnorm;
  int i_ip1;
  double beta1;
  int lastv;
  boolean_T exitg2;
  int exitg1;
  m = A->size[0];
  n = A->size[1];
  knt = A->size[0];
  mn = A->size[1];
  if (knt < mn) {
    mn = knt;
  }

  knt = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity_real_T1(tau, knt);
  if (!((A->size[0] == 0) || (A->size[1] == 0))) {
    emxInit_real_T1(&work, 1);
    i = A->size[1];
    knt = work->size[0];
    work->size[0] = i;
    emxEnsureCapacity_real_T1(work, knt);
    for (knt = 0; knt < i; knt++) {
      work->data[knt] = 0.0;
    }

    for (b_i = 1; b_i <= mn; b_i++) {
      i_i = (b_i + (b_i - 1) * m) - 1;
      mmi = m - b_i;
      if (b_i < m) {
        atmp = A->data[i_i];
        d0 = 0.0;
        if (!(1 + mmi <= 0)) {
          xnorm = xnrm2(mmi, A, i_i + 2);
          if (xnorm != 0.0) {
            beta1 = rt_hypotd_snf(A->data[i_i], xnorm);
            if (A->data[i_i] >= 0.0) {
              beta1 = -beta1;
            }

            if (std::abs(beta1) < 1.0020841800044864E-292) {
              knt = 0;
              do {
                knt++;
                xscal(mmi, 9.9792015476736E+291, A, i_i + 2);
                beta1 *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

              beta1 = rt_hypotd_snf(atmp, xnrm2(mmi, A, i_i + 2));
              if (atmp >= 0.0) {
                beta1 = -beta1;
              }

              d0 = (beta1 - atmp) / beta1;
              xscal(mmi, 1.0 / (atmp - beta1), A, i_i + 2);
              for (i = 1; i <= knt; i++) {
                beta1 *= 1.0020841800044864E-292;
              }

              atmp = beta1;
            } else {
              d0 = (beta1 - A->data[i_i]) / beta1;
              xnorm = 1.0 / (A->data[i_i] - beta1);
              xscal(mmi, xnorm, A, i_i + 2);
              atmp = beta1;
            }
          }
        }

        tau->data[b_i - 1] = d0;
        A->data[i_i] = atmp;
      } else {
        tau->data[b_i - 1] = 0.0;
      }

      if (b_i < n) {
        atmp = A->data[i_i];
        A->data[i_i] = 1.0;
        i_ip1 = b_i + b_i * m;
        if (tau->data[b_i - 1] != 0.0) {
          lastv = 1 + mmi;
          i = i_i + mmi;
          while ((lastv > 0) && (A->data[i] == 0.0)) {
            lastv--;
            i--;
          }

          knt = n - b_i;
          exitg2 = false;
          while ((!exitg2) && (knt > 0)) {
            i = i_ip1 + (knt - 1) * m;
            mmi = i;
            do {
              exitg1 = 0;
              if (mmi <= (i + lastv) - 1) {
                if (A->data[mmi - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  mmi++;
                }
              } else {
                knt--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          knt = 0;
        }

        if (lastv > 0) {
          xgemv(lastv, knt, A, i_ip1, m, A, i_i + 1, work);
          xger(lastv, knt, -tau->data[b_i - 1], i_i + 1, work, A, i_ip1, m);
        }

        A->data[i_i] = atmp;
      }
    }

    emxFree_real_T(&work);
  }
}

//
// File trailer for xgeqrf.cpp
//
// [EOF]
//
