/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mpower.cpp
 *
 * Code generation for function 'mpower'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "mpower.h"
#include "inv.h"

/* Function Definitions */
void mpower(const double a[16], double b, double c[16])
{
  double e;
  double b_a[16];
  int n;
  int i3;
  boolean_T firstmult;
  int cBuffer_tmp;
  int b_cBuffer_tmp;
  int exitg1;
  int nb;
  double ed2;
  boolean_T first;
  boolean_T aBufferInUse;
  double cBuffer[16];
  int i4;
  int k;
  double aBuffer[16];
  if (std::floor(b) == b) {
    e = std::abs(b);
    if (e <= 2.147483647E+9) {
      memcpy(&b_a[0], &a[0], sizeof(double) << 4);
      n = (int)e;
      cBuffer_tmp = (int)e;
      b_cBuffer_tmp = 0;
      nb = -2;
      while (cBuffer_tmp > 0) {
        nb++;
        if ((cBuffer_tmp & 1) != 0) {
          b_cBuffer_tmp++;
        }

        cBuffer_tmp >>= 1;
      }

      if ((int)e <= 2) {
        if (b == 2.0) {
          for (i3 = 0; i3 < 4; i3++) {
            for (i4 = 0; i4 < 4; i4++) {
              cBuffer_tmp = i4 << 2;
              b_cBuffer_tmp = i3 + cBuffer_tmp;
              c[b_cBuffer_tmp] = 0.0;
              c[b_cBuffer_tmp] = ((a[i3] * a[cBuffer_tmp] + a[i3 + 4] * a[1 +
                                   cBuffer_tmp]) + a[i3 + 8] * a[2 + cBuffer_tmp])
                + a[i3 + 12] * a[3 + cBuffer_tmp];
            }
          }
        } else if (b == 1.0) {
          memcpy(&c[0], &a[0], sizeof(double) << 4);
        } else if (b == -1.0) {
          inv(a, c);
        } else if (b == -2.0) {
          for (i3 = 0; i3 < 4; i3++) {
            for (i4 = 0; i4 < 4; i4++) {
              cBuffer_tmp = i4 << 2;
              b_cBuffer_tmp = i3 + cBuffer_tmp;
              b_a[b_cBuffer_tmp] = 0.0;
              b_a[b_cBuffer_tmp] = ((a[i3] * a[cBuffer_tmp] + a[i3 + 4] * a[1 +
                cBuffer_tmp]) + a[i3 + 8] * a[2 + cBuffer_tmp]) + a[i3 + 12] *
                a[3 + cBuffer_tmp];
            }
          }

          inv(b_a, c);
        } else {
          memset(&c[0], 0, sizeof(double) << 4);
          c[0] = 1.0;
          c[5] = 1.0;
          c[10] = 1.0;
          c[15] = 1.0;
        }
      } else {
        first = true;
        aBufferInUse = false;
        firstmult = ((b_cBuffer_tmp & 1) != 0);
        if ((firstmult && (b < 0.0)) || ((!firstmult) && (b >= 0.0))) {
          firstmult = true;
        } else {
          firstmult = false;
        }

        for (k = 0; k <= nb; k++) {
          if ((n & 1) != 0) {
            if (first) {
              first = false;
              if (firstmult) {
                if (aBufferInUse) {
                  memcpy(&cBuffer[0], &aBuffer[0], sizeof(double) << 4);
                } else {
                  memcpy(&cBuffer[0], &b_a[0], sizeof(double) << 4);
                }
              } else if (aBufferInUse) {
                memcpy(&c[0], &aBuffer[0], sizeof(double) << 4);
              } else {
                memcpy(&c[0], &b_a[0], sizeof(double) << 4);
              }
            } else {
              if (aBufferInUse) {
                if (firstmult) {
                  for (i3 = 0; i3 < 4; i3++) {
                    for (i4 = 0; i4 < 4; i4++) {
                      cBuffer_tmp = i4 << 2;
                      b_cBuffer_tmp = i3 + cBuffer_tmp;
                      c[b_cBuffer_tmp] = 0.0;
                      c[b_cBuffer_tmp] = ((cBuffer[i3] * aBuffer[cBuffer_tmp] +
                                           cBuffer[i3 + 4] * aBuffer[1 +
                                           cBuffer_tmp]) + cBuffer[i3 + 8] *
                                          aBuffer[2 + cBuffer_tmp]) + cBuffer[i3
                        + 12] * aBuffer[3 + cBuffer_tmp];
                    }
                  }
                } else {
                  for (i3 = 0; i3 < 4; i3++) {
                    for (i4 = 0; i4 < 4; i4++) {
                      cBuffer_tmp = i4 << 2;
                      b_cBuffer_tmp = i3 + cBuffer_tmp;
                      cBuffer[b_cBuffer_tmp] = 0.0;
                      cBuffer[b_cBuffer_tmp] = ((c[i3] * aBuffer[cBuffer_tmp] +
                        c[i3 + 4] * aBuffer[1 + cBuffer_tmp]) + c[i3 + 8] *
                        aBuffer[2 + cBuffer_tmp]) + c[i3 + 12] * aBuffer[3 +
                        cBuffer_tmp];
                    }
                  }
                }
              } else if (firstmult) {
                for (i3 = 0; i3 < 4; i3++) {
                  for (i4 = 0; i4 < 4; i4++) {
                    cBuffer_tmp = i4 << 2;
                    b_cBuffer_tmp = i3 + cBuffer_tmp;
                    c[b_cBuffer_tmp] = 0.0;
                    c[b_cBuffer_tmp] = ((cBuffer[i3] * b_a[cBuffer_tmp] +
                                         cBuffer[i3 + 4] * b_a[1 + cBuffer_tmp])
                                        + cBuffer[i3 + 8] * b_a[2 + cBuffer_tmp])
                      + cBuffer[i3 + 12] * b_a[3 + cBuffer_tmp];
                  }
                }
              } else {
                for (i3 = 0; i3 < 4; i3++) {
                  for (i4 = 0; i4 < 4; i4++) {
                    cBuffer_tmp = i4 << 2;
                    b_cBuffer_tmp = i3 + cBuffer_tmp;
                    cBuffer[b_cBuffer_tmp] = 0.0;
                    cBuffer[b_cBuffer_tmp] = ((c[i3] * b_a[cBuffer_tmp] + c[i3 +
                      4] * b_a[1 + cBuffer_tmp]) + c[i3 + 8] * b_a[2 +
                      cBuffer_tmp]) + c[i3 + 12] * b_a[3 + cBuffer_tmp];
                  }
                }
              }

              firstmult = !firstmult;
            }
          }

          n >>= 1;
          if (aBufferInUse) {
            for (i3 = 0; i3 < 4; i3++) {
              for (i4 = 0; i4 < 4; i4++) {
                cBuffer_tmp = i4 << 2;
                b_cBuffer_tmp = i3 + cBuffer_tmp;
                b_a[b_cBuffer_tmp] = 0.0;
                b_a[b_cBuffer_tmp] = ((aBuffer[i3] * aBuffer[cBuffer_tmp] +
                  aBuffer[i3 + 4] * aBuffer[1 + cBuffer_tmp]) + aBuffer[i3 + 8] *
                                      aBuffer[2 + cBuffer_tmp]) + aBuffer[i3 +
                  12] * aBuffer[3 + cBuffer_tmp];
              }
            }
          } else {
            for (i3 = 0; i3 < 4; i3++) {
              for (i4 = 0; i4 < 4; i4++) {
                cBuffer_tmp = i4 << 2;
                b_cBuffer_tmp = i3 + cBuffer_tmp;
                aBuffer[b_cBuffer_tmp] = 0.0;
                aBuffer[b_cBuffer_tmp] = ((b_a[i3] * b_a[cBuffer_tmp] + b_a[i3 +
                  4] * b_a[1 + cBuffer_tmp]) + b_a[i3 + 8] * b_a[2 + cBuffer_tmp])
                  + b_a[i3 + 12] * b_a[3 + cBuffer_tmp];
              }
            }
          }

          aBufferInUse = !aBufferInUse;
        }

        if (first) {
          if (b < 0.0) {
            if (aBufferInUse) {
              inv(aBuffer, c);
            } else {
              inv(b_a, c);
            }
          } else if (aBufferInUse) {
            memcpy(&c[0], &aBuffer[0], sizeof(double) << 4);
          } else {
            memcpy(&c[0], &b_a[0], sizeof(double) << 4);
          }
        } else if (b < 0.0) {
          if (aBufferInUse) {
            for (i3 = 0; i3 < 4; i3++) {
              for (i4 = 0; i4 < 4; i4++) {
                cBuffer_tmp = i4 << 2;
                b_cBuffer_tmp = i3 + cBuffer_tmp;
                cBuffer[b_cBuffer_tmp] = 0.0;
                cBuffer[b_cBuffer_tmp] = ((c[i3] * aBuffer[cBuffer_tmp] + c[i3 +
                  4] * aBuffer[1 + cBuffer_tmp]) + c[i3 + 8] * aBuffer[2 +
                  cBuffer_tmp]) + c[i3 + 12] * aBuffer[3 + cBuffer_tmp];
              }
            }
          } else {
            for (i3 = 0; i3 < 4; i3++) {
              for (i4 = 0; i4 < 4; i4++) {
                cBuffer_tmp = i4 << 2;
                b_cBuffer_tmp = i3 + cBuffer_tmp;
                cBuffer[b_cBuffer_tmp] = 0.0;
                cBuffer[b_cBuffer_tmp] = ((c[i3] * b_a[cBuffer_tmp] + c[i3 + 4] *
                  b_a[1 + cBuffer_tmp]) + c[i3 + 8] * b_a[2 + cBuffer_tmp]) +
                  c[i3 + 12] * b_a[3 + cBuffer_tmp];
              }
            }
          }

          inv(cBuffer, c);
        } else if (aBufferInUse) {
          for (i3 = 0; i3 < 4; i3++) {
            for (i4 = 0; i4 < 4; i4++) {
              cBuffer_tmp = i4 << 2;
              b_cBuffer_tmp = i3 + cBuffer_tmp;
              c[b_cBuffer_tmp] = 0.0;
              c[b_cBuffer_tmp] = ((cBuffer[i3] * aBuffer[cBuffer_tmp] +
                                   cBuffer[i3 + 4] * aBuffer[1 + cBuffer_tmp]) +
                                  cBuffer[i3 + 8] * aBuffer[2 + cBuffer_tmp]) +
                cBuffer[i3 + 12] * aBuffer[3 + cBuffer_tmp];
            }
          }
        } else {
          for (i3 = 0; i3 < 4; i3++) {
            for (i4 = 0; i4 < 4; i4++) {
              cBuffer_tmp = i4 << 2;
              b_cBuffer_tmp = i3 + cBuffer_tmp;
              c[b_cBuffer_tmp] = 0.0;
              c[b_cBuffer_tmp] = ((cBuffer[i3] * b_a[cBuffer_tmp] + cBuffer[i3 +
                                   4] * b_a[1 + cBuffer_tmp]) + cBuffer[i3 + 8] *
                                  b_a[2 + cBuffer_tmp]) + cBuffer[i3 + 12] *
                b_a[3 + cBuffer_tmp];
            }
          }
        }
      }
    } else {
      memcpy(&b_a[0], &a[0], sizeof(double) << 4);
      if ((!rtIsInf(b)) && (!rtIsNaN(b))) {
        firstmult = true;
        do {
          exitg1 = 0;
          ed2 = std::floor(e / 2.0);
          if (2.0 * ed2 != e) {
            if (firstmult) {
              memcpy(&c[0], &b_a[0], sizeof(double) << 4);
              firstmult = false;
            } else {
              for (i3 = 0; i3 < 4; i3++) {
                for (i4 = 0; i4 < 4; i4++) {
                  cBuffer_tmp = i4 << 2;
                  b_cBuffer_tmp = i3 + cBuffer_tmp;
                  cBuffer[b_cBuffer_tmp] = 0.0;
                  cBuffer[b_cBuffer_tmp] = ((c[i3] * b_a[cBuffer_tmp] + c[i3 + 4]
                    * b_a[1 + cBuffer_tmp]) + c[i3 + 8] * b_a[2 + cBuffer_tmp])
                    + c[i3 + 12] * b_a[3 + cBuffer_tmp];
                }
              }

              memcpy(&c[0], &cBuffer[0], sizeof(double) << 4);
            }
          }

          if (ed2 == 0.0) {
            exitg1 = 1;
          } else {
            e = ed2;
            for (i3 = 0; i3 < 4; i3++) {
              for (i4 = 0; i4 < 4; i4++) {
                cBuffer_tmp = i4 << 2;
                b_cBuffer_tmp = i3 + cBuffer_tmp;
                cBuffer[b_cBuffer_tmp] = 0.0;
                cBuffer[b_cBuffer_tmp] = ((b_a[i3] * b_a[cBuffer_tmp] + b_a[i3 +
                  4] * b_a[1 + cBuffer_tmp]) + b_a[i3 + 8] * b_a[2 + cBuffer_tmp])
                  + b_a[i3 + 12] * b_a[3 + cBuffer_tmp];
              }
            }

            memcpy(&b_a[0], &cBuffer[0], sizeof(double) << 4);
          }
        } while (exitg1 == 0);

        if (b < 0.0) {
          memcpy(&b_a[0], &c[0], sizeof(double) << 4);
          inv(b_a, c);
        }
      } else {
        for (i3 = 0; i3 < 16; i3++) {
          c[i3] = rtNaN;
        }
      }
    }
  }
}

/* End of code generation (mpower.cpp) */
