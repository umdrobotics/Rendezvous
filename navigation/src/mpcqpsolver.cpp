//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpcqpsolver.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 02-Sep-2018 11:08:41
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
  int m;
  int inner;
  int j;
  int coffset;
  int Hinv_size[2];
  int boffset;
  double a_data[100];
  double Hinv_data[100];
  double lam[68];
  short iA1[68];
  int i;
  int k;
  int aoffset;
  m = Linv_size[1];
  inner = Linv_size[0];
  for (j = 0; j < inner; j++) {
    coffset = Linv_size[1];
    for (boffset = 0; boffset < coffset; boffset++) {
      a_data[boffset + m * j] = Linv_data[j + Linv_size[0] * boffset];
    }
  }

  if (Linv_size[0] == 1) {
    Hinv_size[0] = Linv_size[1];
    Hinv_size[1] = Linv_size[1];
    inner = Linv_size[1];
    for (j = 0; j < inner; j++) {
      coffset = Linv_size[1];
      for (boffset = 0; boffset < coffset; boffset++) {
        Hinv_data[j + Hinv_size[0] * boffset] = 0.0;
        for (m = 0; m < 1; m++) {
          Hinv_data[j + Hinv_size[0] * boffset] += a_data[j] * Linv_data[boffset];
        }
      }
    }
  } else {
    m = Linv_size[1];
    inner = Linv_size[0];
    Hinv_size[0] = Linv_size[1];
    Hinv_size[1] = Linv_size[1];
    for (j = 0; j < Linv_size[1]; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 1; i <= m; i++) {
        Hinv_data[(coffset + i) - 1] = 0.0;
      }

      for (k = 0; k < inner; k++) {
        if (Linv_data[boffset + k] != 0.0) {
          aoffset = k * m;
          for (i = 0; i < m; i++) {
            Hinv_data[coffset + i] += Linv_data[boffset + k] * a_data[aoffset +
              i];
          }
        }
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
