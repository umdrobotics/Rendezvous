//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpower.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "mpower.h"
#include "matrix_to_integer_power.h"

// Function Definitions

//
// Arguments    : const double a[16]
//                double b
//                double c[16]
// Return Type  : void
//
void mpower(const double a[16], double b, double c[16])
{
  if (std::floor(b) == b) {
    matrix_to_integer_power(a, b, c);
  }
}

//
// File trailer for mpower.cpp
//
// [EOF]
//
