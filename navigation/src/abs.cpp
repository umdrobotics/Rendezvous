/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * abs.cpp
 *
 * Code generation for function 'abs'
 *
 */

/* Include files */
#include <cmath>
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/abs.h"

/* Function Definitions */
void b_abs(const double x[10], double y[10])
{
  int k;
  for (k = 0; k < 10; k++) {
    y[k] = std::abs(x[k]);
  }
}

void c_abs(const double x[68], double y[68])
{
  int k;
  for (k = 0; k < 68; k++) {
    y[k] = std::abs(x[k]);
  }
}

/* End of code generation (abs.cpp) */
