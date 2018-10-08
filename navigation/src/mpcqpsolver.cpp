//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpcqpsolver.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
//

// Include Files
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/mpcqpsolver.h"
#include "navigation/qpkwik.h"

// Function Definitions

//
// Arguments    : const double Linv_data[]
//                const int Linv_size[2]
//                const double f[10]
//                const double A[680]
//                const double b[68]
//                double x_data[]
//                int x_size[1]
//                double *status
// Return Type  : void
//
void mpcqpsolver(const double Linv_data[], const int Linv_size[2], const double
                 f[10], const double A[680], const double b[68], double x_data[],
                 int x_size[1], double *status)
{
  int a_size_idx_0;
  int loop_ub;
  int i4;
  int br;
  int k;
  int Hinv_size[2];
  int i5;
  double a_data[100];
  int m;
  double Hinv_data[100];
  double lam[68];
  short iA1[68];
  int ic;
  int ar;
  int ib;
  int ia;
  a_size_idx_0 = Linv_size[1];
  loop_ub = Linv_size[0];
  for (i4 = 0; i4 < loop_ub; i4++) {
    br = Linv_size[1];
    for (i5 = 0; i5 < br; i5++) {
      a_data[i5 + a_size_idx_0 * i4] = Linv_data[i4 + Linv_size[0] * i5];
    }
  }

  if (Linv_size[0] == 1) {
    Hinv_size[0] = Linv_size[1];
    Hinv_size[1] = Linv_size[1];
    loop_ub = Linv_size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      br = Linv_size[1];
      for (i5 = 0; i5 < br; i5++) {
        Hinv_data[i4 + Hinv_size[0] * i5] = 0.0;
        for (a_size_idx_0 = 0; a_size_idx_0 < 1; a_size_idx_0++) {
          Hinv_data[i4 + Hinv_size[0] * i5] += a_data[i4] * Linv_data[i5];
        }
      }
    }
  } else {
    k = Linv_size[0];
    Hinv_size[0] = (signed char)Linv_size[1];
    Hinv_size[1] = (signed char)Linv_size[1];
    m = Linv_size[1];
    loop_ub = (signed char)Linv_size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      br = Hinv_size[0];
      for (i5 = 0; i5 < br; i5++) {
        Hinv_data[i5 + Hinv_size[0] * i4] = 0.0;
      }
    }

    if (Linv_size[1] == 0) {
    } else {
      a_size_idx_0 = Linv_size[1] * (Linv_size[1] - 1);
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= a_size_idx_0)) {
        i4 = loop_ub + m;
        for (ic = loop_ub; ic + 1 <= i4; ic++) {
          Hinv_data[ic] = 0.0;
        }

        loop_ub += m;
      }

      br = 0;
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= a_size_idx_0)) {
        ar = 0;
        i4 = br + k;
        for (ib = br; ib + 1 <= i4; ib++) {
          if (Linv_data[ib] != 0.0) {
            ia = ar;
            i5 = loop_ub + m;
            for (ic = loop_ub; ic + 1 <= i5; ic++) {
              ia++;
              Hinv_data[ic] += Linv_data[ib] * a_data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        loop_ub += m;
      }
    }
  }

  qpkwik(Linv_data, Linv_size, Hinv_data, Hinv_size, f, A, b, x_data, x_size,
         lam, status, iA1);
}

//
// File trailer for mpcqpsolver.cpp
//
// [EOF]
//
