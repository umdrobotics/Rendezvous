//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: abs.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
//

// Include Files
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/abs.h"

// Function Definitions

//
// Arguments    : const double x[10]
//                double y[10]
// Return Type  : void
//
void b_abs(const double x[10], double y[10])
{
  int k;
  for (k = 0; k < 10; k++) {
    y[k] = std::abs(x[k]);
  }
}

//
// Arguments    : const double x[68]
//                double y[68]
// Return Type  : void
//
void c_abs(const double x[68], double y[68])
{
  int k;
  for (k = 0; k < 68; k++) {
    y[k] = std::abs(x[k]);
  }
}

//
// File trailer for abs.cpp
//
// [EOF]
//
