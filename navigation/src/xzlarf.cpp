/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzlarf.cpp
 *
 * Code generation for function 'xzlarf'
 *
 */

/* Include files */
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/xzlarf.h"
#include "navigation/xger.h"
#include "navigation/xgemv.h"

/* Function Definitions */
void b_xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0, int
              ldc, double work_data[])
{
  int lastv;
  int lastc;
  boolean_T exitg2;
  int coltop;
  int ia;
  int exitg1;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = ic0 + (lastc - 1) * ldc;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
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
    xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0, work_data);
    xger(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

void xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0, int ldc,
            double work_data[])
{
  int lastv;
  int lastc;
  boolean_T exitg2;
  int coltop;
  int ia;
  int exitg1;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = ic0 + (lastc - 1) * ldc;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
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
    xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0, work_data);
    xger(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

/* End of code generation (xzlarf.cpp) */
