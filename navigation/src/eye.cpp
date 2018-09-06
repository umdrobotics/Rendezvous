//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
//

// Include Files
#include <string.h>
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/eye.h"

// Function Definitions

//
// Arguments    : double I[16]
// Return Type  : void
//
void eye(double I[16])
{
  int k;
  memset(&I[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    I[k + (k << 2)] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
