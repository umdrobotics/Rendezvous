//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: matrix_to_integer_power.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "matrix_to_integer_power.h"
#include "inv.h"

// Function Definitions

//
// Arguments    : const double a[16]
//                double b
//                double c[16]
// Return Type  : void
//
void matrix_to_integer_power(const double a[16], double b, double c[16])
{
  double b_a[16];
  double e;
  int n;
  int b_n;
  boolean_T firstmult;
  int nbitson;
  int exitg1;
  int nb;
  double ed2;
  boolean_T first;
  boolean_T aBufferInUse;
  double cBuffer[16];
  int i2;
  int k;
  double aBuffer[16];
  if (2.147483647E+9 >= std::abs(b)) {
    memcpy(&b_a[0], &a[0], sizeof(double) << 4);
    e = std::abs(b);
    b_n = (int)std::abs(b);
    n = (int)e;
    nbitson = 0;
    nb = -1;
    while (n > 0) {
      nb++;
      if ((n & 1) != 0) {
        nbitson++;
      }

      n >>= 1;
    }

    if ((int)e <= 2) {
      if (b == 2.0) {
        for (n = 0; n < 4; n++) {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            c[n + (nbitson << 2)] = 0.0;
            for (i2 = 0; i2 < 4; i2++) {
              c[n + (nbitson << 2)] += a[n + (i2 << 2)] * a[i2 + (nbitson << 2)];
            }
          }
        }
      } else if (b == 1.0) {
        memcpy(&c[0], &a[0], sizeof(double) << 4);
      } else if (b == -1.0) {
        inv(a, c);
      } else if (b == -2.0) {
        for (n = 0; n < 4; n++) {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            b_a[n + (nbitson << 2)] = 0.0;
            for (i2 = 0; i2 < 4; i2++) {
              b_a[n + (nbitson << 2)] += a[n + (i2 << 2)] * a[i2 + (nbitson << 2)];
            }
          }
        }

        inv(b_a, c);
      } else {
        memset(&c[0], 0, sizeof(double) << 4);
        for (n = 0; n < 4; n++) {
          c[n + (n << 2)] = 1.0;
        }
      }
    } else {
      first = true;
      aBufferInUse = false;
      firstmult = ((nbitson & 1) != 0);
      if ((firstmult && (b < 0.0)) || ((!firstmult) && (b >= 0.0))) {
        firstmult = true;
      } else {
        firstmult = false;
      }

      for (k = 1; k <= nb; k++) {
        if ((b_n & 1) != 0) {
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
                for (n = 0; n < 4; n++) {
                  for (nbitson = 0; nbitson < 4; nbitson++) {
                    c[n + (nbitson << 2)] = 0.0;
                    for (i2 = 0; i2 < 4; i2++) {
                      c[n + (nbitson << 2)] += cBuffer[n + (i2 << 2)] *
                        aBuffer[i2 + (nbitson << 2)];
                    }
                  }
                }
              } else {
                for (n = 0; n < 4; n++) {
                  for (nbitson = 0; nbitson < 4; nbitson++) {
                    cBuffer[n + (nbitson << 2)] = 0.0;
                    for (i2 = 0; i2 < 4; i2++) {
                      cBuffer[n + (nbitson << 2)] += c[n + (i2 << 2)] *
                        aBuffer[i2 + (nbitson << 2)];
                    }
                  }
                }
              }
            } else if (firstmult) {
              for (n = 0; n < 4; n++) {
                for (nbitson = 0; nbitson < 4; nbitson++) {
                  c[n + (nbitson << 2)] = 0.0;
                  for (i2 = 0; i2 < 4; i2++) {
                    c[n + (nbitson << 2)] += cBuffer[n + (i2 << 2)] * b_a[i2 +
                      (nbitson << 2)];
                  }
                }
              }
            } else {
              for (n = 0; n < 4; n++) {
                for (nbitson = 0; nbitson < 4; nbitson++) {
                  cBuffer[n + (nbitson << 2)] = 0.0;
                  for (i2 = 0; i2 < 4; i2++) {
                    cBuffer[n + (nbitson << 2)] += c[n + (i2 << 2)] * b_a[i2 +
                      (nbitson << 2)];
                  }
                }
              }
            }

            firstmult = !firstmult;
          }
        }

        b_n >>= 1;
        if (aBufferInUse) {
          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              b_a[n + (nbitson << 2)] = 0.0;
              for (i2 = 0; i2 < 4; i2++) {
                b_a[n + (nbitson << 2)] += aBuffer[n + (i2 << 2)] * aBuffer[i2 +
                  (nbitson << 2)];
              }
            }
          }
        } else {
          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              aBuffer[n + (nbitson << 2)] = 0.0;
              for (i2 = 0; i2 < 4; i2++) {
                aBuffer[n + (nbitson << 2)] += b_a[n + (i2 << 2)] * b_a[i2 +
                  (nbitson << 2)];
              }
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
          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              cBuffer[n + (nbitson << 2)] = 0.0;
              for (i2 = 0; i2 < 4; i2++) {
                cBuffer[n + (nbitson << 2)] += c[n + (i2 << 2)] * aBuffer[i2 +
                  (nbitson << 2)];
              }
            }
          }
        } else {
          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              cBuffer[n + (nbitson << 2)] = 0.0;
              for (i2 = 0; i2 < 4; i2++) {
                cBuffer[n + (nbitson << 2)] += c[n + (i2 << 2)] * b_a[i2 +
                  (nbitson << 2)];
              }
            }
          }
        }

        inv(cBuffer, c);
      } else if (aBufferInUse) {
        for (n = 0; n < 4; n++) {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            c[n + (nbitson << 2)] = 0.0;
            for (i2 = 0; i2 < 4; i2++) {
              c[n + (nbitson << 2)] += cBuffer[n + (i2 << 2)] * aBuffer[i2 +
                (nbitson << 2)];
            }
          }
        }
      } else {
        for (n = 0; n < 4; n++) {
          for (nbitson = 0; nbitson < 4; nbitson++) {
            c[n + (nbitson << 2)] = 0.0;
            for (i2 = 0; i2 < 4; i2++) {
              c[n + (nbitson << 2)] += cBuffer[n + (i2 << 2)] * b_a[i2 +
                (nbitson << 2)];
            }
          }
        }
      }
    }
  } else {
    memcpy(&b_a[0], &a[0], sizeof(double) << 4);
    if ((!rtIsInf(b)) && (!rtIsNaN(b))) {
      e = std::abs(b);
      firstmult = true;
      do {
        exitg1 = 0;
        ed2 = std::floor(e / 2.0);
        if (2.0 * ed2 != e) {
          if (firstmult) {
            memcpy(&c[0], &b_a[0], sizeof(double) << 4);
            firstmult = false;
          } else {
            for (n = 0; n < 4; n++) {
              for (nbitson = 0; nbitson < 4; nbitson++) {
                cBuffer[n + (nbitson << 2)] = 0.0;
                for (i2 = 0; i2 < 4; i2++) {
                  cBuffer[n + (nbitson << 2)] += c[n + (i2 << 2)] * b_a[i2 +
                    (nbitson << 2)];
                }
              }
            }

            for (n = 0; n < 4; n++) {
              for (nbitson = 0; nbitson < 4; nbitson++) {
                c[nbitson + (n << 2)] = cBuffer[nbitson + (n << 2)];
              }
            }
          }
        }

        if (ed2 == 0.0) {
          exitg1 = 1;
        } else {
          e = ed2;
          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              cBuffer[n + (nbitson << 2)] = 0.0;
              for (i2 = 0; i2 < 4; i2++) {
                cBuffer[n + (nbitson << 2)] += b_a[n + (i2 << 2)] * b_a[i2 +
                  (nbitson << 2)];
              }
            }
          }

          for (n = 0; n < 4; n++) {
            for (nbitson = 0; nbitson < 4; nbitson++) {
              b_a[nbitson + (n << 2)] = cBuffer[nbitson + (n << 2)];
            }
          }
        }
      } while (exitg1 == 0);

      if (b < 0.0) {
        memcpy(&cBuffer[0], &c[0], sizeof(double) << 4);
        inv(cBuffer, c);
      }
    } else {
      for (n = 0; n < 16; n++) {
        c[n] = rtNaN;
      }
    }
  }
}

//
// File trailer for matrix_to_integer_power.cpp
//
// [EOF]
//
