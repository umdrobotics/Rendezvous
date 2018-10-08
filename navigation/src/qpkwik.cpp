//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpkwik.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 08-Oct-2018 14:40:09
//

// Include Files
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/qpkwik.h"
#include "navigation/qr.h"
#include "navigation/abs.h"
#include "navigation/norm.h"

// Function Declarations
static void DropConstraint(short kDrop, short iA[68], short *nA, short iC[68]);
static double KWIKfactor(const double Ac[680], const short iC[68], short nA,
  const double Linv_data[], const int Linv_size[2], double RLinv_data[], int
  RLinv_size[2], double D_data[], int D_size[2], double H_data[], int H_size[2]);
static void Unconstrained(const double Hinv_data[], const int Hinv_size[2],
  const double f[10], double x_data[]);

// Function Definitions

//
// Arguments    : short kDrop
//                short iA[68]
//                short *nA
//                short iC[68]
// Return Type  : void
//
static void DropConstraint(short kDrop, short iA[68], short *nA, short iC[68])
{
  int i11;
  short i12;
  short i;
  iA[iC[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    i11 = *nA - 1;
    if (i11 < -32768) {
      i11 = -32768;
    }

    i12 = (short)i11;
    for (i = kDrop; i <= i12; i++) {
      iC[i - 1] = iC[i];
    }
  }

  iC[*nA - 1] = 0;
  i11 = *nA - 1;
  if (i11 < -32768) {
    i11 = -32768;
  }

  *nA = (short)i11;
}

//
// Arguments    : const double Ac[680]
//                const short iC[68]
//                short nA
//                const double Linv_data[]
//                const int Linv_size[2]
//                double RLinv_data[]
//                int RLinv_size[2]
//                double D_data[]
//                int D_size[2]
//                double H_data[]
//                int H_size[2]
// Return Type  : double
//
static double KWIKfactor(const double Ac[680], const short iC[68], short nA,
  const double Linv_data[], const int Linv_size[2], double RLinv_data[], int
  RLinv_size[2], double D_data[], int D_size[2], double H_data[], int H_size[2])
{
  double Status;
  int TL_size_idx_0;
  int ia;
  int loop_ub;
  int i7;
  short i;
  int ar;
  double QQ_data[100];
  int QQ_size[2];
  double RR_data[100];
  int RR_size[2];
  int k;
  int b_size_idx_0;
  int m;
  int exitg1;
  double b_data[10];
  int br;
  int ic;
  short j;
  double a_data[10];
  double y;
  short b_k;
  double TL_data[100];
  TL_size_idx_0 = Linv_size[0];
  Status = 1.0;
  ia = RLinv_size[0];
  loop_ub = RLinv_size[1];
  for (i7 = 0; i7 < loop_ub; i7++) {
    for (ar = 0; ar < ia; ar++) {
      RLinv_data[ar + RLinv_size[0] * i7] = 0.0;
    }
  }

  for (i = 1; i <= nA; i++) {
    if (Linv_size[1] == 1) {
      b_size_idx_0 = Linv_size[0];
      ia = Linv_size[0];
      for (i7 = 0; i7 < ia; i7++) {
        b_data[i7] = 0.0;
        for (ar = 0; ar < 1; ar++) {
          b_data[i7] += Linv_data[i7] * Ac[iC[i - 1] - 1];
        }
      }
    } else {
      k = Linv_size[1];
      m = Linv_size[0];
      ar = (signed char)Linv_size[0];
      b_size_idx_0 = (signed char)Linv_size[0];
      if (0 <= ar - 1) {
        memset(&b_data[0], 0, (unsigned int)(ar * (int)sizeof(double)));
      }

      if (Linv_size[0] != 0) {
        ar = 0;
        while ((m > 0) && (ar <= 0)) {
          for (ic = 1; ic <= m; ic++) {
            b_data[ic - 1] = 0.0;
          }

          ar = m;
        }

        br = 0;
        ar = 0;
        while ((m > 0) && (ar <= 0)) {
          ar = -1;
          i7 = br + k;
          for (loop_ub = br; loop_ub + 1 <= i7; loop_ub++) {
            if (Ac[(iC[i - 1] + 68 * loop_ub) - 1] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                b_data[ic] += Ac[(iC[i - 1] + 68 * loop_ub) - 1] * Linv_data[ia];
              }
            }

            ar += m;
          }

          br += k;
          ar = m;
        }
      }
    }

    for (i7 = 0; i7 < b_size_idx_0; i7++) {
      RLinv_data[i7 + RLinv_size[0] * (i - 1)] = b_data[i7];
    }
  }

  qr(RLinv_data, RLinv_size, QQ_data, QQ_size, RR_data, RR_size);
  i = 1;
  do {
    exitg1 = 0;
    if (i <= nA) {
      if (std::abs(RR_data[(i + RR_size[0] * (i - 1)) - 1]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      ia = Linv_size[0];
      ar = Linv_size[0];
      loop_ub = QQ_size[0];
      for (i = 0; i < 10; i++) {
        for (i7 = 0; i7 < ia; i7++) {
          a_data[i7] = Linv_data[i7 + Linv_size[0] * i];
        }

        for (j = 0; j < 10; j++) {
          for (i7 = 0; i7 < loop_ub; i7++) {
            b_data[i7] = QQ_data[i7 + QQ_size[0] * j];
          }

          if ((ia == 1) || (QQ_size[0] == 1)) {
            y = 0.0;
            for (i7 = 0; i7 < ar; i7++) {
              y += a_data[i7] * b_data[i7];
            }
          } else {
            y = 0.0;
            for (i7 = 0; i7 < ar; i7++) {
              y += a_data[i7] * b_data[i7];
            }
          }

          TL_data[i + TL_size_idx_0 * j] = y;
        }
      }

      ia = RLinv_size[0];
      loop_ub = RLinv_size[1];
      for (i7 = 0; i7 < loop_ub; i7++) {
        for (ar = 0; ar < ia; ar++) {
          RLinv_data[ar + RLinv_size[0] * i7] = 0.0;
        }
      }

      for (j = nA; j > 0; j--) {
        RLinv_data[(j + RLinv_size[0] * (j - 1)) - 1] = 1.0;
        for (b_k = j; b_k <= nA; b_k++) {
          RLinv_data[(j + RLinv_size[0] * (b_k - 1)) - 1] /= RR_data[(j +
            RR_size[0] * (j - 1)) - 1];
        }

        if (j > 1) {
          for (i = 1; i < j; i++) {
            for (b_k = j; b_k <= nA; b_k++) {
              RLinv_data[(i + RLinv_size[0] * (b_k - 1)) - 1] -= RR_data[(i +
                RR_size[0] * (j - 1)) - 1] * RLinv_data[(j + RLinv_size[0] *
                (b_k - 1)) - 1];
            }
          }
        }
      }

      for (i = 0; i < 10; i++) {
        for (j = (short)(i + 1); j < 11; j++) {
          H_data[i + H_size[0] * (j - 1)] = 0.0;
          for (b_k = (short)(nA + 1); b_k < 11; b_k++) {
            H_data[i + H_size[0] * (j - 1)] -= TL_data[i + TL_size_idx_0 * (b_k
              - 1)] * TL_data[(j + TL_size_idx_0 * (b_k - 1)) - 1];
          }

          H_data[(j + H_size[0] * i) - 1] = H_data[i + H_size[0] * (j - 1)];
        }
      }

      for (j = 1; j <= nA; j++) {
        for (i = 0; i < 10; i++) {
          D_data[i + D_size[0] * (j - 1)] = 0.0;
          for (b_k = j; b_k <= nA; b_k++) {
            D_data[i + D_size[0] * (j - 1)] += TL_data[i + TL_size_idx_0 * (b_k
              - 1)] * RLinv_data[(j + RLinv_size[0] * (b_k - 1)) - 1];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

//
// Arguments    : const double Hinv_data[]
//                const int Hinv_size[2]
//                const double f[10]
//                double x_data[]
// Return Type  : void
//
static void Unconstrained(const double Hinv_data[], const int Hinv_size[2],
  const double f[10], double x_data[])
{
  int loop_ub;
  short i;
  int i13;
  double d2;
  double b_Hinv_data[10];
  loop_ub = Hinv_size[1];
  for (i = 0; i < 10; i++) {
    for (i13 = 0; i13 < loop_ub; i13++) {
      b_Hinv_data[i13] = -Hinv_data[i + Hinv_size[0] * i13];
    }

    d2 = 0.0;
    for (i13 = 0; i13 < 10; i13++) {
      d2 += b_Hinv_data[i13] * f[i13];
    }

    x_data[i] = d2;
  }
}

//
// Arguments    : const double Linv_data[]
//                const int Linv_size[2]
//                const double Hinv_data[]
//                const int Hinv_size[2]
//                const double f[10]
//                const double Ac[680]
//                const double b[68]
//                double x_data[]
//                int x_size[1]
//                double lambda[68]
//                double *status
//                short iA[68]
// Return Type  : void
//
void qpkwik(const double Linv_data[], const int Linv_size[2], const double
            Hinv_data[], const int Hinv_size[2], const double f[10], const
            double Ac[680], const double b[68], double x_data[], int x_size[1],
            double lambda[68], double *status, short iA[68])
{
  double r_data[10];
  double rMin;
  int RLinv_size[2];
  int D_size[2];
  int H_size[2];
  boolean_T cTolComputed;
  int i;
  short nA;
  double cTol[68];
  short iC[68];
  double Xnorm0;
  int exitg2;
  double cMin;
  short kNext;
  short b_i;
  double b_x_data[10];
  int i6;
  int exitg1;
  double d0;
  double b_Ac[10];
  double cVal;
  boolean_T guard1 = false;
  double RLinv_data[100];
  double D_data[100];
  double H_data[100];
  double varargin_1[68];
  int k;
  int z_size_idx_0;
  int ix;
  int m;
  boolean_T exitg3;
  double a_data[100];
  double z_data[10];
  short kDrop;
  double t1;
  boolean_T isT1Inf;
  boolean_T tempOK;
  int br;
  int ic;
  short iSave;
  int ia;
  *status = 1.0;
  memset(&iA[0], 0, 68U * sizeof(short));
  memset(&lambda[0], 0, 68U * sizeof(double));
  memset(&r_data[0], 0, 10U * sizeof(double));
  rMin = 0.0;
  RLinv_size[0] = Linv_size[0];
  RLinv_size[1] = Linv_size[1];
  D_size[0] = Linv_size[0];
  D_size[1] = Linv_size[1];
  H_size[0] = Linv_size[0];
  H_size[1] = Linv_size[1];
  cTolComputed = false;
  for (i = 0; i < 68; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  x_size[0] = 10;
  memset(&x_data[0], 0, 10U * sizeof(double));
  Unconstrained(Hinv_data, Hinv_size, f, x_data);
  Xnorm0 = norm(x_data);
  do {
    exitg2 = 0;
    if (*status <= 200.0) {
      cMin = -1.0E-6;
      kNext = 0;
      for (b_i = 0; b_i < 68; b_i++) {
        if (!cTolComputed) {
          memcpy(&b_x_data[0], &x_data[0], 10U * sizeof(double));
          for (i6 = 0; i6 < 10; i6++) {
            b_Ac[i6] = Ac[b_i + 68 * i6] * b_x_data[i6];
          }

          b_abs(b_Ac, b_x_data);
          i = 1;
          cVal = b_x_data[0];
          if (rtIsNaN(b_x_data[0])) {
            ix = 2;
            exitg3 = false;
            while ((!exitg3) && (ix < 11)) {
              i = ix;
              if (!rtIsNaN(b_x_data[ix - 1])) {
                cVal = b_x_data[ix - 1];
                exitg3 = true;
              } else {
                ix++;
              }
            }
          }

          if (i < 10) {
            while (i + 1 < 11) {
              if (b_x_data[i] > cVal) {
                cVal = b_x_data[i];
              }

              i++;
            }
          }

          if ((cTol[b_i] > cVal) || rtIsNaN(cVal)) {
          } else {
            cTol[b_i] = cVal;
          }
        }

        if (iA[b_i] == 0) {
          d0 = 0.0;
          for (i6 = 0; i6 < 10; i6++) {
            d0 += Ac[b_i + 68 * i6] * x_data[i6];
          }

          cVal = (d0 - b[b_i]) / cTol[b_i];
          if (cVal < cMin) {
            cMin = cVal;
            kNext = (short)(b_i + 1);
          }
        }
      }

      cTolComputed = true;
      if (kNext <= 0) {
        exitg2 = 1;
      } else {
        do {
          exitg1 = 0;
          if ((kNext > 0) && (*status <= 200.0)) {
            guard1 = false;
            if (nA == 0) {
              if (Hinv_size[1] == 1) {
                z_size_idx_0 = Hinv_size[0];
                i = Hinv_size[0];
                for (i6 = 0; i6 < i; i6++) {
                  z_data[i6] = 0.0;
                  for (ix = 0; ix < 1; ix++) {
                    z_data[i6] += Hinv_data[i6] * Ac[kNext - 1];
                  }
                }
              } else {
                k = Hinv_size[1];
                m = Hinv_size[0];
                i = (signed char)Hinv_size[0];
                z_size_idx_0 = (signed char)Hinv_size[0];
                if (0 <= i - 1) {
                  memset(&z_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
                }

                if (Hinv_size[0] != 0) {
                  i = 0;
                  while ((m > 0) && (i <= 0)) {
                    for (ic = 1; ic <= m; ic++) {
                      z_data[ic - 1] = 0.0;
                    }

                    i = m;
                  }

                  br = 0;
                  i = 0;
                  while ((m > 0) && (i <= 0)) {
                    i = -1;
                    i6 = br + k;
                    for (ix = br; ix + 1 <= i6; ix++) {
                      if (Ac[(kNext + 68 * ix) - 1] != 0.0) {
                        ia = i;
                        for (ic = 0; ic + 1 <= m; ic++) {
                          ia++;
                          z_data[ic] += Ac[(kNext + 68 * ix) - 1] * Hinv_data[ia];
                        }
                      }

                      i += m;
                    }

                    br += k;
                    i = m;
                  }
                }
              }

              guard1 = true;
            } else {
              cVal = KWIKfactor(Ac, iC, nA, Linv_data, Linv_size, RLinv_data,
                                RLinv_size, D_data, D_size, H_data, H_size);
              if (cVal <= 0.0) {
                *status = -2.0;
                exitg1 = 1;
              } else {
                i = H_size[0] * H_size[1];
                for (i6 = 0; i6 < i; i6++) {
                  a_data[i6] = -H_data[i6];
                }

                if (H_size[1] == 1) {
                  z_size_idx_0 = H_size[0];
                  i = H_size[0];
                  for (i6 = 0; i6 < i; i6++) {
                    z_data[i6] = 0.0;
                    for (ix = 0; ix < 1; ix++) {
                      z_data[i6] += a_data[i6] * Ac[kNext - 1];
                    }
                  }
                } else {
                  k = H_size[1];
                  m = H_size[0];
                  i = (signed char)H_size[0];
                  z_size_idx_0 = (signed char)H_size[0];
                  if (0 <= i - 1) {
                    memset(&z_data[0], 0, (unsigned int)(i * (int)sizeof(double)));
                  }

                  if (H_size[0] != 0) {
                    i = 0;
                    while ((m > 0) && (i <= 0)) {
                      for (ic = 1; ic <= m; ic++) {
                        z_data[ic - 1] = 0.0;
                      }

                      i = m;
                    }

                    br = 0;
                    i = 0;
                    while ((m > 0) && (i <= 0)) {
                      i = -1;
                      i6 = br + k;
                      for (ix = br; ix + 1 <= i6; ix++) {
                        if (Ac[(kNext + 68 * ix) - 1] != 0.0) {
                          ia = i;
                          for (ic = 0; ic + 1 <= m; ic++) {
                            ia++;
                            z_data[ic] += Ac[(kNext + 68 * ix) - 1] * a_data[ia];
                          }
                        }

                        i += m;
                      }

                      br += k;
                      i = m;
                    }
                  }
                }

                for (b_i = 1; b_i <= nA; b_i++) {
                  i = D_size[0];
                  for (i6 = 0; i6 < i; i6++) {
                    b_x_data[i6] = D_data[i6 + D_size[0] * (b_i - 1)];
                  }

                  d0 = 0.0;
                  for (i6 = 0; i6 < 10; i6++) {
                    d0 += Ac[(kNext + 68 * i6) - 1] * b_x_data[i6];
                  }

                  r_data[b_i - 1] = d0;
                }

                guard1 = true;
              }
            }

            if (guard1) {
              kDrop = 0;
              t1 = 0.0;
              isT1Inf = true;
              tempOK = true;
              if (nA > 0) {
                iSave = 1;
                exitg3 = false;
                while ((!exitg3) && (iSave <= nA)) {
                  if (r_data[iSave - 1] >= 1.0E-12) {
                    tempOK = false;
                    exitg3 = true;
                  } else {
                    iSave++;
                  }
                }
              }

              if ((nA == 0) || tempOK) {
                tempOK = true;
              } else {
                tempOK = false;
              }

              if (!tempOK) {
                for (b_i = 1; b_i <= nA; b_i++) {
                  if (r_data[b_i - 1] > 1.0E-12) {
                    cVal = lambda[iC[b_i - 1] - 1] / r_data[b_i - 1];
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = b_i;
                    }
                  }
                }

                if (kDrop > 0) {
                  t1 = rMin;
                  isT1Inf = false;
                }
              }

              if (0 <= z_size_idx_0 - 1) {
                memcpy(&b_x_data[0], &z_data[0], (unsigned int)(z_size_idx_0 *
                        (int)sizeof(double)));
              }

              d0 = 0.0;
              for (i6 = 0; i6 < 10; i6++) {
                d0 += b_x_data[i6] * Ac[(kNext + 68 * i6) - 1];
              }

              if (d0 <= 0.0) {
                cMin = 0.0;
                tempOK = true;
              } else {
                cVal = 0.0;
                for (i6 = 0; i6 < 10; i6++) {
                  cVal += Ac[(kNext + 68 * i6) - 1] * x_data[i6];
                }

                cMin = (b[kNext - 1] - cVal) / d0;
                tempOK = false;
              }

              if (isT1Inf && tempOK) {
                *status = -1.0;
                exitg1 = 1;
              } else {
                if ((t1 < cMin) || rtIsNaN(cMin)) {
                  cVal = t1;
                } else {
                  cVal = cMin;
                }

                if (tempOK) {
                  cVal = t1;
                } else {
                  if (isT1Inf) {
                    cVal = cMin;
                  }
                }

                for (b_i = 1; b_i <= nA; b_i++) {
                  lambda[iC[b_i - 1] - 1] -= cVal * r_data[b_i - 1];
                  if (lambda[iC[b_i - 1] - 1] < 0.0) {
                    lambda[iC[b_i - 1] - 1] = 0.0;
                  }
                }

                lambda[kNext - 1] += cVal;
                if (cVal == t1) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!tempOK) {
                  x_size[0] = 10;
                  for (i6 = 0; i6 < 10; i6++) {
                    x_data[i6] += cVal * z_data[i6];
                  }

                  if (cVal == cMin) {
                    if (nA == 10) {
                      *status = -1.0;
                      exitg1 = 1;
                    } else {
                      i6 = nA + 1;
                      if (i6 > 32767) {
                        i6 = 32767;
                      }

                      nA = (short)i6;
                      iC[nA - 1] = kNext;
                      b_i = nA;
                      while ((b_i > 1) && (!(iC[b_i - 1] > iC[b_i - 2]))) {
                        iSave = iC[b_i - 1];
                        iC[b_i - 1] = iC[b_i - 2];
                        iC[b_i - 2] = iSave;
                        b_i--;
                      }

                      iA[kNext - 1] = 1;
                      kNext = 0;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cVal = norm(x_data);
            if (std::abs(cVal - Xnorm0) > 0.001) {
              Xnorm0 = cVal;
              c_abs(b, varargin_1);
              for (k = 0; k < 68; k++) {
                cVal = varargin_1[k];
                if (!(cVal > 1.0)) {
                  cVal = 1.0;
                }

                cTol[k] = cVal;
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = 1;
        }
      }
    } else {
      *status = 0.0;
      exitg2 = 1;
    }
  } while (exitg2 == 0);
}

//
// File trailer for qpkwik.cpp
//
// [EOF]
//
