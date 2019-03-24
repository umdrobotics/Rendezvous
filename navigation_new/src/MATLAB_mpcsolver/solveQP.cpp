//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solveQP.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "mpower.h"
#include "solveQP_emxutil.h"
#include "xscal.h"
#include "mpcqpsolver.h"
#include "inv.h"
#include "eye.h"

// Function Definitions

//
// input
//  P = 12;
//  M = 5;
//  % rp = repmat([100,100,0,0]', P ,1);
//  % xk = [0,0,0,0]';
//
//  q = 0.99;
// Arguments    : double P
//                double M
//                const double xk[4]
//                const emxArray_real_T *rp
//                double q
//                double k
//                double Qf
//                double Qb
//                emxArray_real_T *x
// Return Type  : void
//
void solveQP(double P, double M, const double xk[4], const emxArray_real_T *rp,
             double q, double k, double Qf, double Qb, emxArray_real_T *x)
{
  emxArray_real_T *Q;
  int i0;
  int jmax;
  int nmj;
  int ar;
  emxArray_real_T *H;
  double a;
  emxArray_real_T *Ap;
  double ajj;
  double Ap0[16];
  emxArray_real_T *Bp;
  int i1;
  double c;
  double b_Ap0[16];
  int n;
  static const double b[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.025,
    0.0, 0.9994, 0.0, 0.0, 0.025, 0.0, 0.9994 };

  emxArray_real_T *Bpj;
  emxArray_real_T *Cv;
  emxArray_real_T *b_a;
  double Bpi[8];
  static const double B[8] = { -0.0031, 0.0, -0.2451, 0.0, 0.0, -0.0031, 0.0,
    -0.2451 };

  int b_k;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int j;
  int m;
  emxArray_real_T *C;
  int iv0[4];
  double b_b[8];
  int br;
  int ix;
  int jj;
  int ia;
  emxArray_real_T *b_C;
  emxArray_real_T *y;
  emxArray_real_T *c_a;
  emxArray_real_T *G;
  emxArray_real_T *b_H;
  emxArray_int32_T *r0;
  emxArray_real_T *S;
  emxArray_int8_T *d_a;
  emxArray_int32_T *r1;
  emxArray_real_T *b_y;
  emxArray_real_T *c_C;
  boolean_T exitg1;
  emxArray_boolean_T *r2;
  emxInit_real_T(&Q, 2);
  eye(4.0 * P, Q);
  i0 = Q->size[0] * Q->size[1];
  emxEnsureCapacity_real_T(Q, i0);
  jmax = Q->size[0];
  nmj = Q->size[1];
  ar = jmax * nmj;
  for (i0 = 0; i0 < ar; i0++) {
    Q->data[i0] *= q;
  }

  emxInit_real_T(&H, 2);
  a = 0.35 * (1.0 - q);
  eye(2.0 * M, H);

  //
  //  k = 7;
  for (nmj = 0; nmj < (int)(k - 1.0); nmj++) {
    Q->data[((int)(4.0 * (1.0 + (double)nmj) + 1.0) + Q->size[0] * ((int)(4.0 *
               (1.0 + (double)nmj) + 1.0) - 1)) - 1] = Qf;

    // 10;
    Q->data[((int)(4.0 * (1.0 + (double)nmj) + 2.0) + Q->size[0] * ((int)(4.0 *
               (1.0 + (double)nmj) + 2.0) - 1)) - 1] = Qf;

    // 10;
  }

  i0 = (int)((P - 1.0) + (1.0 - k));
  for (nmj = 0; nmj < i0; nmj++) {
    ajj = k + (double)nmj;
    Q->data[((int)(4.0 * ajj + 1.0) + Q->size[0] * ((int)(4.0 * ajj + 1.0) - 1))
      - 1] = Qb;

    // 1.15;
    Q->data[((int)(4.0 * ajj + 2.0) + Q->size[0] * ((int)(4.0 * ajj + 2.0) - 1))
      - 1] = Qb;

    // 1.15;
  }

  emxInit_real_T(&Ap, 2);

  //
  //      H = [1 -1; -1 2];
  //      f = [-2; -6];
  //      A = [1 1; -1 2; 2 1];
  //      b = [2; 2; 3];
  //      lb = [0; 0];
  //      [L,p] = chol(H,'lower');
  //      Linv = inv(L);
  //      opt = mpcqpsolverOptions;
  //      iA0 = false(size(b));
  //      Aeq = [];
  //      beq = zeros(0,1);
  //  %     [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb);
  //      [x,status] = mpcqpsolver(Linv,f,A,b,Aeq,beq,iA0,opt);
  //  A = [1    0   0.0498  0.0001;
  //       0    1   -0.0002   0.0495;
  //       0.0009    0.0008   0.9895   0.0038;
  //       -0.0011    -0.0009   -0.0064   0.9792 ];
  //
  //  B = [0.0003     -0.0006;
  //       -0.0001    0.0004;
  //       -0.0035    -0.0051;
  //       0.0132     -0.0049];
  //  Build Ap
  i0 = Ap->size[0] * Ap->size[1];
  Ap->size[0] = (int)(4.0 * P);
  Ap->size[1] = 4;
  emxEnsureCapacity_real_T(Ap, i0);
  ar = (int)(4.0 * P) << 2;
  for (i0 = 0; i0 < ar; i0++) {
    Ap->data[i0] = 0.0;
  }

  b_eye(Ap0);
  for (nmj = 0; nmj < (int)P; nmj++) {
    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        b_Ap0[i0 + (i1 << 2)] = 0.0;
        for (n = 0; n < 4; n++) {
          b_Ap0[i0 + (i1 << 2)] += Ap0[i0 + (n << 2)] * b[n + (i1 << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        Ap0[i1 + (i0 << 2)] = b_Ap0[i1 + (i0 << 2)];
      }
    }

    c = ((1.0 + (double)nmj) - 1.0) * 4.0;
    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        Ap->data[((int)(c + (1.0 + (double)i1)) + Ap->size[0] * i0) - 1] =
          Ap0[i1 + (i0 << 2)];
      }
    }
  }

  emxInit_real_T(&Bp, 2);

  //  Build Bp
  i0 = Bp->size[0] * Bp->size[1];
  Bp->size[0] = (int)(4.0 * P);
  Bp->size[1] = (int)(2.0 * M);
  emxEnsureCapacity_real_T(Bp, i0);
  ar = (int)(4.0 * P) * (int)(2.0 * M);
  for (i0 = 0; i0 < ar; i0++) {
    Bp->data[i0] = 0.0;
  }

  nmj = 0;
  emxInit_real_T(&Bpj, 2);
  while (nmj <= (int)P - 1) {
    i0 = Bpj->size[0] * Bpj->size[1];
    Bpj->size[0] = 4;
    Bpj->size[1] = (int)(2.0 * M);
    emxEnsureCapacity_real_T(Bpj, i0);
    ar = (int)(2.0 * M) << 2;
    for (i0 = 0; i0 < ar; i0++) {
      Bpj->data[i0] = 0.0;
    }

    if (1.0 + (double)nmj < M) {
      memcpy(&Bpi[0], &B[0], sizeof(double) << 3);
    } else {
      mpower(b, (1.0 + (double)nmj) - M, Ap0);
      for (i0 = 0; i0 < 4; i0++) {
        for (i1 = 0; i1 < 2; i1++) {
          Bpi[i0 + (i1 << 2)] = 0.0;
          for (n = 0; n < 4; n++) {
            Bpi[i0 + (i1 << 2)] += Ap0[i0 + (n << 2)] * B[n + (i1 << 2)];
          }
        }
      }
    }

    ajj = 1.0 + (double)nmj;
    if (M < ajj) {
      ajj = M;
    }

    i0 = (int)((1.0 + (-1.0 - ajj)) / -1.0);
    for (j = 0; j < i0; j++) {
      c = ((ajj + -(double)j) - 1.0) * 2.0;
      for (i1 = 0; i1 < 2; i1++) {
        for (n = 0; n < 4; n++) {
          Bpj->data[n + Bpj->size[0] * ((int)(c + (1.0 + (double)i1)) - 1)] =
            Bpi[n + (i1 << 2)];
        }
      }

      for (i1 = 0; i1 < 4; i1++) {
        for (n = 0; n < 2; n++) {
          b_b[i1 + (n << 2)] = 0.0;
          for (br = 0; br < 4; br++) {
            b_b[i1 + (n << 2)] += b[i1 + (br << 2)] * Bpi[br + (n << 2)];
          }
        }
      }

      for (i1 = 0; i1 < 2; i1++) {
        for (n = 0; n < 4; n++) {
          Bpi[n + (i1 << 2)] = b_b[n + (i1 << 2)];
        }
      }
    }

    c = ((1.0 + (double)nmj) - 1.0) * 4.0;
    for (i0 = 0; i0 < 4; i0++) {
      iv0[i0] = (int)(c + (1.0 + (double)i0)) - 1;
    }

    ar = Bpj->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        Bp->data[iv0[i1] + Bp->size[0] * i0] = Bpj->data[i1 + Bpj->size[0] * i0];
      }
    }

    //                  disp(Bp);
    nmj++;
  }

  emxInit_real_T(&Cv, 2);
  emxInit_real_T(&b_a, 2);
  i0 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = Bp->size[1];
  b_a->size[1] = Bp->size[0];
  emxEnsureCapacity_real_T(b_a, i0);
  ar = Bp->size[0];
  for (i0 = 0; i0 < ar; i0++) {
    jmax = Bp->size[1];
    for (i1 = 0; i1 < jmax; i1++) {
      b_a->data[i1 + b_a->size[0] * i0] = Bp->data[i0 + Bp->size[0] * i1];
    }
  }

  if ((b_a->size[1] == 1) || (Q->size[0] == 1)) {
    i0 = Cv->size[0] * Cv->size[1];
    Cv->size[0] = b_a->size[0];
    Cv->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(Cv, i0);
    ar = b_a->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = Q->size[1];
      for (i1 = 0; i1 < jmax; i1++) {
        Cv->data[i0 + Cv->size[0] * i1] = 0.0;
        nmj = b_a->size[1];
        for (n = 0; n < nmj; n++) {
          Cv->data[i0 + Cv->size[0] * i1] += b_a->data[i0 + b_a->size[0] * n] *
            Q->data[n + Q->size[0] * i1];
        }
      }
    }
  } else {
    b_k = b_a->size[1];
    unnamed_idx_0 = (unsigned int)b_a->size[0];
    unnamed_idx_1 = (unsigned int)Q->size[1];
    i0 = Cv->size[0] * Cv->size[1];
    Cv->size[0] = (int)unnamed_idx_0;
    Cv->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(Cv, i0);
    m = b_a->size[0];
    i0 = Cv->size[0] * Cv->size[1];
    emxEnsureCapacity_real_T(Cv, i0);
    ar = Cv->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = Cv->size[0];
      for (i1 = 0; i1 < jmax; i1++) {
        Cv->data[i1 + Cv->size[0] * i0] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (Q->size[1] == 0)) {
    } else {
      nmj = b_a->size[0] * (Q->size[1] - 1);
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i0 = jmax + m;
        for (ix = jmax; ix + 1 <= i0; ix++) {
          Cv->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Q->data[jj] != 0.0) {
            ia = ar;
            i1 = jmax + m;
            for (ix = jmax; ix + 1 <= i1; ix++) {
              ia++;
              Cv->data[ix] += Q->data[jj] * b_a->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  emxInit_real_T(&C, 2);
  if ((Cv->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = C->size[0] * C->size[1];
    C->size[0] = Cv->size[0];
    C->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(C, i0);
    ar = Cv->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = Bp->size[1];
      for (i1 = 0; i1 < jmax; i1++) {
        C->data[i0 + C->size[0] * i1] = 0.0;
        nmj = Cv->size[1];
        for (n = 0; n < nmj; n++) {
          C->data[i0 + C->size[0] * i1] += Cv->data[i0 + Cv->size[0] * n] *
            Bp->data[n + Bp->size[0] * i1];
        }
      }
    }
  } else {
    b_k = Cv->size[1];
    unnamed_idx_0 = (unsigned int)Cv->size[0];
    unnamed_idx_1 = (unsigned int)Bp->size[1];
    i0 = C->size[0] * C->size[1];
    C->size[0] = (int)unnamed_idx_0;
    C->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(C, i0);
    m = Cv->size[0];
    i0 = C->size[0] * C->size[1];
    emxEnsureCapacity_real_T(C, i0);
    ar = C->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = C->size[0];
      for (i1 = 0; i1 < jmax; i1++) {
        C->data[i1 + C->size[0] * i0] = 0.0;
      }
    }

    if ((Cv->size[0] == 0) || (Bp->size[1] == 0)) {
    } else {
      nmj = Cv->size[0] * (Bp->size[1] - 1);
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i0 = jmax + m;
        for (ix = jmax; ix + 1 <= i0; ix++) {
          C->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Bp->data[jj] != 0.0) {
            ia = ar;
            i1 = jmax + m;
            for (ix = jmax; ix + 1 <= i1; ix++) {
              ia++;
              C->data[ix] += Bp->data[jj] * Cv->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  i0 = H->size[0] * H->size[1];
  emxEnsureCapacity_real_T(H, i0);
  jmax = H->size[0];
  nmj = H->size[1];
  ar = jmax * nmj;
  for (i0 = 0; i0 < ar; i0++) {
    H->data[i0] = a * H->data[i0] + C->data[i0];
  }

  i0 = Bpj->size[0] * Bpj->size[1];
  Bpj->size[0] = 4;
  Bpj->size[1] = Ap->size[0];
  emxEnsureCapacity_real_T(Bpj, i0);
  ar = Ap->size[0];
  for (i0 = 0; i0 < ar; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      Bpj->data[i1 + Bpj->size[0] * i0] = Ap->data[i0 + Ap->size[0] * i1];
    }
  }

  emxInit_real_T(&b_C, 2);
  unnamed_idx_1 = (unsigned int)Bpj->size[1];
  i0 = b_C->size[0] * b_C->size[1];
  b_C->size[1] = (int)unnamed_idx_1;
  emxEnsureCapacity_real_T(b_C, i0);
  n = Bpj->size[1] - 1;
  i0 = b_C->size[0] * b_C->size[1];
  b_C->size[0] = 1;
  emxEnsureCapacity_real_T(b_C, i0);
  ar = b_C->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    b_C->data[b_C->size[0] * i0] = 0.0;
  }

  if (Bpj->size[1] != 0) {
    for (jmax = 1; jmax - 1 <= n; jmax++) {
      for (ix = jmax; ix <= jmax; ix++) {
        b_C->data[ix - 1] = 0.0;
      }
    }

    br = 0;
    for (jmax = 0; jmax <= n; jmax++) {
      ar = -1;
      for (jj = br; jj + 1 <= br + 4; jj++) {
        if (Bpj->data[jj] != 0.0) {
          ia = ar;
          for (ix = jmax; ix + 1 <= jmax + 1; ix++) {
            ia++;
            b_C->data[ix] += Bpj->data[jj] * xk[ia];
          }
        }

        ar++;
      }

      br += 4;
    }
  }

  emxFree_real_T(&Bpj);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&c_a, 2);
  i0 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = 1;
  c_a->size[1] = b_C->size[1];
  emxEnsureCapacity_real_T(c_a, i0);
  ar = b_C->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    c_a->data[c_a->size[0] * i0] = b_C->data[b_C->size[0] * i0] - rp->data[i0];
  }

  if ((c_a->size[1] == 1) || (Q->size[0] == 1)) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(y, i0);
    ar = Q->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      y->data[y->size[0] * i0] = 0.0;
      jmax = c_a->size[1];
      for (i1 = 0; i1 < jmax; i1++) {
        y->data[y->size[0] * i0] += c_a->data[c_a->size[0] * i1] * Q->data[i1 +
          Q->size[0] * i0];
      }
    }
  } else {
    b_k = c_a->size[1];
    unnamed_idx_1 = (unsigned int)Q->size[1];
    i0 = y->size[0] * y->size[1];
    y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(y, i0);
    n = Q->size[1] - 1;
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    emxEnsureCapacity_real_T(y, i0);
    ar = y->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      y->data[y->size[0] * i0] = 0.0;
    }

    if (Q->size[1] != 0) {
      for (jmax = 1; jmax - 1 <= n; jmax++) {
        for (ix = jmax; ix <= jmax; ix++) {
          y->data[ix - 1] = 0.0;
        }
      }

      br = 0;
      for (jmax = 0; jmax <= n; jmax++) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Q->data[jj] != 0.0) {
            ia = ar;
            for (ix = jmax; ix + 1 <= jmax + 1; ix++) {
              ia++;
              y->data[ix] += Q->data[jj] * c_a->data[ia];
            }
          }

          ar++;
        }

        br += b_k;
      }
    }
  }

  emxFree_real_T(&c_a);
  if ((y->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = 1;
    b_C->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(b_C, i0);
    ar = Bp->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      b_C->data[b_C->size[0] * i0] = 0.0;
      jmax = y->size[1];
      for (i1 = 0; i1 < jmax; i1++) {
        b_C->data[b_C->size[0] * i0] += y->data[y->size[0] * i1] * Bp->data[i1 +
          Bp->size[0] * i0];
      }
    }
  } else {
    b_k = y->size[1];
    unnamed_idx_1 = (unsigned int)Bp->size[1];
    i0 = b_C->size[0] * b_C->size[1];
    b_C->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(b_C, i0);
    n = Bp->size[1] - 1;
    i0 = b_C->size[0] * b_C->size[1];
    b_C->size[0] = 1;
    emxEnsureCapacity_real_T(b_C, i0);
    ar = b_C->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      b_C->data[b_C->size[0] * i0] = 0.0;
    }

    if (Bp->size[1] != 0) {
      for (jmax = 1; jmax - 1 <= n; jmax++) {
        for (ix = jmax; ix <= jmax; ix++) {
          b_C->data[ix - 1] = 0.0;
        }
      }

      br = 0;
      for (jmax = 0; jmax <= n; jmax++) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Bp->data[jj] != 0.0) {
            ia = ar;
            for (ix = jmax; ix + 1 <= jmax + 1; ix++) {
              ia++;
              b_C->data[ix] += Bp->data[jj] * y->data[ia];
            }
          }

          ar++;
        }

        br += b_k;
      }
    }
  }

  emxFree_real_T(&y);
  i0 = Cv->size[0] * Cv->size[1];
  Cv->size[0] = (int)(2.0 * P);
  Cv->size[1] = (int)(4.0 * P);
  emxEnsureCapacity_real_T(Cv, i0);
  ar = (int)(2.0 * P) * (int)(4.0 * P);
  for (i0 = 0; i0 < ar; i0++) {
    Cv->data[i0] = 0.0;
  }

  for (nmj = 0; nmj < (int)((P - 1.0) + 1.0); nmj++) {
    Cv->data[((int)(2.0 * (double)nmj + 1.0) + Cv->size[0] * ((int)(4.0 *
                (double)nmj + 3.0) - 1)) - 1] = 1.0;
    Cv->data[((int)(2.0 * (double)nmj + 2.0) + Cv->size[0] * ((int)(4.0 *
                (double)nmj + 4.0) - 1)) - 1] = 1.0;
  }

  emxInit_real_T(&G, 2);
  i0 = G->size[0] * G->size[1];
  G->size[0] = (int)(4.0 * P + 4.0 * M);
  G->size[1] = (int)(2.0 * M);
  emxEnsureCapacity_real_T(G, i0);
  ar = (int)(4.0 * P + 4.0 * M) * (int)(2.0 * M);
  for (i0 = 0; i0 < ar; i0++) {
    G->data[i0] = 0.0;
  }

  emxInit_real_T(&b_H, 2);
  c_eye(2.0 * M, 2.0 * M, b_H);
  ar = b_H->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    jmax = b_H->size[0];
    for (i1 = 0; i1 < jmax; i1++) {
      G->data[i1 + G->size[0] * i0] = b_H->data[i1 + b_H->size[0] * i0];
    }
  }

  c = 2.0 * M + 1.0;
  if (c > 4.0 * M) {
    i0 = 0;
  } else {
    i0 = (int)c - 1;
  }

  c_eye(2.0 * M, 2.0 * M, Q);
  i1 = Q->size[0] * Q->size[1];
  emxEnsureCapacity_real_T(Q, i1);
  jmax = Q->size[0];
  nmj = Q->size[1];
  ar = jmax * nmj;
  for (i1 = 0; i1 < ar; i1++) {
    Q->data[i1] = -Q->data[i1];
  }

  ar = Q->size[1];
  for (i1 = 0; i1 < ar; i1++) {
    jmax = Q->size[0];
    for (n = 0; n < jmax; n++) {
      G->data[(i0 + n) + G->size[0] * i1] = Q->data[n + Q->size[0] * i1];
    }
  }

  emxInit_int32_T(&r0, 1);
  c = 4.0 * M;
  ajj = 2.0 * P;
  i0 = r0->size[0];
  r0->size[0] = (int)std::floor(ajj - 1.0) + 1;
  emxEnsureCapacity_int32_T(r0, i0);
  ar = (int)std::floor(ajj - 1.0);
  for (i0 = 0; i0 <= ar; i0++) {
    r0->data[i0] = (int)(c + (1.0 + (double)i0)) - 1;
  }

  if ((Cv->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = C->size[0] * C->size[1];
    C->size[0] = Cv->size[0];
    C->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(C, i0);
    ar = Cv->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = Bp->size[1];
      for (i1 = 0; i1 < jmax; i1++) {
        C->data[i0 + C->size[0] * i1] = 0.0;
        nmj = Cv->size[1];
        for (n = 0; n < nmj; n++) {
          C->data[i0 + C->size[0] * i1] += Cv->data[i0 + Cv->size[0] * n] *
            Bp->data[n + Bp->size[0] * i1];
        }
      }
    }
  } else {
    b_k = Cv->size[1];
    unnamed_idx_0 = (unsigned int)Cv->size[0];
    unnamed_idx_1 = (unsigned int)Bp->size[1];
    i0 = C->size[0] * C->size[1];
    C->size[0] = (int)unnamed_idx_0;
    C->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(C, i0);
    m = Cv->size[0];
    i0 = C->size[0] * C->size[1];
    emxEnsureCapacity_real_T(C, i0);
    ar = C->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = C->size[0];
      for (i1 = 0; i1 < jmax; i1++) {
        C->data[i1 + C->size[0] * i0] = 0.0;
      }
    }

    if ((Cv->size[0] == 0) || (Bp->size[1] == 0)) {
    } else {
      nmj = Cv->size[0] * (Bp->size[1] - 1);
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i0 = jmax + m;
        for (ix = jmax; ix + 1 <= i0; ix++) {
          C->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Bp->data[jj] != 0.0) {
            ia = ar;
            i1 = jmax + m;
            for (ix = jmax; ix + 1 <= i1; ix++) {
              ia++;
              C->data[ix] += Bp->data[jj] * (double)(signed char)Cv->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  ar = C->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    jmax = C->size[0];
    for (i1 = 0; i1 < jmax; i1++) {
      G->data[r0->data[i1] + G->size[0] * i0] = C->data[i1 + C->size[0] * i0];
    }
  }

  emxFree_int32_T(&r0);
  c = (2.0 * P + 4.0 * M) + 1.0;
  if (c > 4.0 * P + 4.0 * M) {
    i0 = 0;
  } else {
    i0 = (int)c - 1;
  }

  i1 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = Cv->size[0];
  b_a->size[1] = Cv->size[1];
  emxEnsureCapacity_real_T(b_a, i1);
  ar = Cv->size[0] * Cv->size[1];
  for (i1 = 0; i1 < ar; i1++) {
    b_a->data[i1] = -Cv->data[i1];
  }

  if ((b_a->size[1] == 1) || (Bp->size[0] == 1)) {
    i1 = C->size[0] * C->size[1];
    C->size[0] = b_a->size[0];
    C->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(C, i1);
    ar = b_a->size[0];
    for (i1 = 0; i1 < ar; i1++) {
      jmax = Bp->size[1];
      for (n = 0; n < jmax; n++) {
        C->data[i1 + C->size[0] * n] = 0.0;
        nmj = b_a->size[1];
        for (br = 0; br < nmj; br++) {
          C->data[i1 + C->size[0] * n] += b_a->data[i1 + b_a->size[0] * br] *
            Bp->data[br + Bp->size[0] * n];
        }
      }
    }
  } else {
    b_k = b_a->size[1];
    unnamed_idx_0 = (unsigned int)b_a->size[0];
    unnamed_idx_1 = (unsigned int)Bp->size[1];
    i1 = C->size[0] * C->size[1];
    C->size[0] = (int)unnamed_idx_0;
    C->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(C, i1);
    m = b_a->size[0];
    i1 = C->size[0] * C->size[1];
    emxEnsureCapacity_real_T(C, i1);
    ar = C->size[1];
    for (i1 = 0; i1 < ar; i1++) {
      jmax = C->size[0];
      for (n = 0; n < jmax; n++) {
        C->data[n + C->size[0] * i1] = 0.0;
      }
    }

    if ((b_a->size[0] == 0) || (Bp->size[1] == 0)) {
    } else {
      nmj = b_a->size[0] * (Bp->size[1] - 1);
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i1 = jmax + m;
        for (ix = jmax; ix + 1 <= i1; ix++) {
          C->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i1 = br + b_k;
        for (jj = br; jj + 1 <= i1; jj++) {
          if (Bp->data[jj] != 0.0) {
            ia = ar;
            n = jmax + m;
            for (ix = jmax; ix + 1 <= n; ix++) {
              ia++;
              C->data[ix] += Bp->data[jj] * (double)(signed char)b_a->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  emxFree_real_T(&b_a);
  emxFree_real_T(&Bp);
  ar = C->size[1];
  for (i1 = 0; i1 < ar; i1++) {
    jmax = C->size[0];
    for (n = 0; n < jmax; n++) {
      G->data[(i0 + n) + G->size[0] * i1] = C->data[n + C->size[0] * i1];
    }
  }

  emxFree_real_T(&C);
  i0 = G->size[0] * G->size[1];
  emxEnsureCapacity_real_T(G, i0);
  jmax = G->size[0];
  nmj = G->size[1];
  ar = jmax * nmj;
  for (i0 = 0; i0 < ar; i0++) {
    G->data[i0] = -G->data[i0];
  }

  emxInit_real_T1(&S, 1);
  i0 = S->size[0];
  S->size[0] = (int)(4.0 * P + 4.0 * M);
  emxEnsureCapacity_real_T1(S, i0);
  ar = (int)(4.0 * P + 4.0 * M);
  for (i0 = 0; i0 < ar; i0++) {
    S->data[i0] = 0.0;
  }

  c = 4.0 * M;
  if (1.0 > c) {
    ar = 0;
  } else {
    ar = (int)c;
  }

  emxInit_int8_T(&d_a, 1);
  i0 = d_a->size[0];
  d_a->size[0] = (int)(4.0 * M);
  emxEnsureCapacity_int8_T(d_a, i0);
  jmax = (int)(4.0 * M);
  for (i0 = 0; i0 < jmax; i0++) {
    d_a->data[i0] = 1;
  }

  emxInit_int32_T1(&r1, 2);
  i0 = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = ar;
  emxEnsureCapacity_int32_T1(r1, i0);
  for (i0 = 0; i0 < ar; i0++) {
    r1->data[r1->size[0] * i0] = i0;
  }

  ar = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    S->data[r1->data[i0]] = (signed char)(d_a->data[i0] * 20);
  }

  i0 = d_a->size[0];
  d_a->size[0] = (int)(2.0 * P);
  emxEnsureCapacity_int8_T(d_a, i0);
  ar = (int)(2.0 * P);
  for (i0 = 0; i0 < ar; i0++) {
    d_a->data[i0] = 1;
  }

  emxInit_real_T(&b_y, 2);
  if ((Cv->size[1] == 1) || (Ap->size[0] == 1)) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = Cv->size[0];
    b_y->size[1] = 4;
    emxEnsureCapacity_real_T(b_y, i0);
    ar = Cv->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        b_y->data[i0 + b_y->size[0] * i1] = 0.0;
        jmax = Cv->size[1];
        for (n = 0; n < jmax; n++) {
          b_y->data[i0 + b_y->size[0] * i1] += Cv->data[i0 + Cv->size[0] * n] *
            Ap->data[n + Ap->size[0] * i1];
        }
      }
    }
  } else {
    b_k = Cv->size[1];
    unnamed_idx_0 = (unsigned int)Cv->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity_real_T(b_y, i0);
    m = Cv->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[1] = 4;
    emxEnsureCapacity_real_T(b_y, i0);
    for (i0 = 0; i0 < 4; i0++) {
      ar = b_y->size[0];
      for (i1 = 0; i1 < ar; i1++) {
        b_y->data[i1 + b_y->size[0] * i0] = 0.0;
      }
    }

    if (Cv->size[0] != 0) {
      nmj = Cv->size[0] * 3;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i0 = jmax + m;
        for (ix = jmax; ix + 1 <= i0; ix++) {
          b_y->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Ap->data[jj] != 0.0) {
            ia = ar;
            i1 = jmax + m;
            for (ix = jmax; ix + 1 <= i1; ix++) {
              ia++;
              b_y->data[ix] += Ap->data[jj] * (double)(signed char)Cv->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  emxInit_real_T1(&c_C, 1);
  unnamed_idx_0 = (unsigned int)b_y->size[0];
  i0 = c_C->size[0];
  c_C->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_real_T1(c_C, i0);
  m = b_y->size[0];
  jmax = c_C->size[0];
  i0 = c_C->size[0];
  c_C->size[0] = jmax;
  emxEnsureCapacity_real_T1(c_C, i0);
  for (i0 = 0; i0 < jmax; i0++) {
    c_C->data[i0] = 0.0;
  }

  if (b_y->size[0] != 0) {
    jmax = 0;
    while ((m > 0) && (jmax <= 0)) {
      for (ix = 1; ix <= m; ix++) {
        c_C->data[ix - 1] = 0.0;
      }

      jmax = m;
    }

    br = 0;
    jmax = 0;
    while ((m > 0) && (jmax <= 0)) {
      ar = -1;
      for (jj = br; jj + 1 <= br + 4; jj++) {
        if (xk[jj] != 0.0) {
          ia = ar;
          for (ix = 0; ix + 1 <= m; ix++) {
            ia++;
            c_C->data[ix] += xk[jj] * b_y->data[ia];
          }
        }

        ar += m;
      }

      br += 4;
      jmax = m;
    }
  }

  c = 4.0 * M;
  ajj = 2.0 * P;
  i0 = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = (int)std::floor(ajj - 1.0) + 1;
  emxEnsureCapacity_int32_T1(r1, i0);
  ar = (int)std::floor(ajj - 1.0);
  for (i0 = 0; i0 <= ar; i0++) {
    r1->data[r1->size[0] * i0] = (int)(c + (1.0 + (double)i0));
  }

  ar = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    S->data[r1->data[i0] - 1] = (double)d_a->data[i0] * 17.0 - c_C->data[i0];
  }

  i0 = d_a->size[0];
  d_a->size[0] = (int)(2.0 * P);
  emxEnsureCapacity_int8_T(d_a, i0);
  ar = (int)(2.0 * P);
  for (i0 = 0; i0 < ar; i0++) {
    d_a->data[i0] = 1;
  }

  if ((Cv->size[1] == 1) || (Ap->size[0] == 1)) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = Cv->size[0];
    b_y->size[1] = 4;
    emxEnsureCapacity_real_T(b_y, i0);
    ar = Cv->size[0];
    for (i0 = 0; i0 < ar; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        b_y->data[i0 + b_y->size[0] * i1] = 0.0;
        jmax = Cv->size[1];
        for (n = 0; n < jmax; n++) {
          b_y->data[i0 + b_y->size[0] * i1] += Cv->data[i0 + Cv->size[0] * n] *
            Ap->data[n + Ap->size[0] * i1];
        }
      }
    }
  } else {
    b_k = Cv->size[1];
    unnamed_idx_0 = (unsigned int)Cv->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity_real_T(b_y, i0);
    m = Cv->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[1] = 4;
    emxEnsureCapacity_real_T(b_y, i0);
    for (i0 = 0; i0 < 4; i0++) {
      ar = b_y->size[0];
      for (i1 = 0; i1 < ar; i1++) {
        b_y->data[i1 + b_y->size[0] * i0] = 0.0;
      }
    }

    if (Cv->size[0] != 0) {
      nmj = Cv->size[0] * 3;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        i0 = jmax + m;
        for (ix = jmax; ix + 1 <= i0; ix++) {
          b_y->data[ix] = 0.0;
        }

        jmax += m;
      }

      br = 0;
      jmax = 0;
      while ((m > 0) && (jmax <= nmj)) {
        ar = -1;
        i0 = br + b_k;
        for (jj = br; jj + 1 <= i0; jj++) {
          if (Ap->data[jj] != 0.0) {
            ia = ar;
            i1 = jmax + m;
            for (ix = jmax; ix + 1 <= i1; ix++) {
              ia++;
              b_y->data[ix] += Ap->data[jj] * (double)(signed char)Cv->data[ia];
            }
          }

          ar += m;
        }

        br += b_k;
        jmax += m;
      }
    }
  }

  emxFree_real_T(&Cv);
  emxFree_real_T(&Ap);
  unnamed_idx_0 = (unsigned int)b_y->size[0];
  i0 = c_C->size[0];
  c_C->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_real_T1(c_C, i0);
  m = b_y->size[0];
  jmax = c_C->size[0];
  i0 = c_C->size[0];
  c_C->size[0] = jmax;
  emxEnsureCapacity_real_T1(c_C, i0);
  for (i0 = 0; i0 < jmax; i0++) {
    c_C->data[i0] = 0.0;
  }

  if (b_y->size[0] != 0) {
    jmax = 0;
    while ((m > 0) && (jmax <= 0)) {
      for (ix = 1; ix <= m; ix++) {
        c_C->data[ix - 1] = 0.0;
      }

      jmax = m;
    }

    br = 0;
    jmax = 0;
    while ((m > 0) && (jmax <= 0)) {
      ar = -1;
      for (jj = br; jj + 1 <= br + 4; jj++) {
        if (xk[jj] != 0.0) {
          ia = ar;
          for (ix = 0; ix + 1 <= m; ix++) {
            ia++;
            c_C->data[ix] += xk[jj] * b_y->data[ia];
          }
        }

        ar += m;
      }

      br += 4;
      jmax = m;
    }
  }

  emxFree_real_T(&b_y);
  c = (4.0 * M + 1.0) + 2.0 * P;
  ajj = 4.0 * M + 4.0 * P;
  if (c > ajj) {
    i0 = 0;
    i1 = 0;
  } else {
    i0 = (int)c - 1;
    i1 = (int)ajj;
  }

  n = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = i1 - i0;
  emxEnsureCapacity_int32_T1(r1, n);
  ar = i1 - i0;
  for (i1 = 0; i1 < ar; i1++) {
    r1->data[r1->size[0] * i1] = i0 + i1;
  }

  ar = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    S->data[r1->data[i0]] = (double)d_a->data[i0] * 17.0 + c_C->data[i0];
  }

  emxFree_int8_T(&d_a);
  emxFree_int32_T(&r1);
  i0 = S->size[0];
  emxEnsureCapacity_real_T1(S, i0);
  ar = S->size[0];
  for (i0 = 0; i0 < ar; i0++) {
    S->data[i0] = -S->data[i0];
  }

  //  G = eye(2*M, 2*M);
  //  G = [G ; -eye(2*M, 2*M)];
  //  G = -G;
  //  S = ones(4*M,1)*20;
  //  S = -S;
  //  G = Cv*Bp;
  //  G = [G ; -Cv*Bp];
  //
  //  S = ones(2*P,1)*17 - Cv*Ap*xk;
  //  S = [S; ones(2*P,1)*17 + Cv*Ap*xk];
  //  ub = ones(2*M,1)*20;
  //  lb = ones(2*M,1)*(-20);
  n = H->size[1];
  if (H->size[1] != 0) {
    br = H->size[0];
    ar = 0;
    if (H->size[0] != 0) {
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j + 1 <= br)) {
        jj = j + j * n;
        ajj = 0.0;
        if (!(j < 1)) {
          ix = j;
          m = j;
          for (b_k = 1; b_k <= j; b_k++) {
            ajj += H->data[ix] * H->data[m];
            ix += n;
            m += n;
          }
        }

        ajj = H->data[jj] - ajj;
        if (ajj > 0.0) {
          ajj = std::sqrt(ajj);
          H->data[jj] = ajj;
          if (j + 1 < br) {
            nmj = (br - j) - 1;
            if ((nmj == 0) || (j == 0)) {
            } else {
              ix = j;
              i0 = (j + n * (j - 1)) + 2;
              for (jmax = j + 2; jmax <= i0; jmax += n) {
                c = -H->data[ix];
                m = jj + 1;
                i1 = (jmax + nmj) - 1;
                for (ia = jmax; ia <= i1; ia++) {
                  H->data[m] += H->data[ia - 1] * c;
                  m++;
                }

                ix += n;
              }
            }

            xscal(nmj, 1.0 / ajj, H, jj + 2);
          }

          j++;
        } else {
          H->data[jj] = ajj;
          ar = j + 1;
          exitg1 = true;
        }
      }
    }

    if (ar == 0) {
      jmax = n;
    } else {
      jmax = ar - 1;
    }

    for (j = 1; j + 1 <= jmax; j++) {
      for (nmj = 1; nmj <= j; nmj++) {
        H->data[(nmj + H->size[0] * j) - 1] = 0.0;
      }
    }

    if (1 > jmax) {
      ar = 0;
      jmax = 0;
    } else {
      ar = jmax;
    }

    i0 = b_H->size[0] * b_H->size[1];
    b_H->size[0] = ar;
    b_H->size[1] = jmax;
    emxEnsureCapacity_real_T(b_H, i0);
    for (i0 = 0; i0 < jmax; i0++) {
      for (i1 = 0; i1 < ar; i1++) {
        b_H->data[i1 + b_H->size[0] * i0] = H->data[i1 + H->size[0] * i0];
      }
    }

    i0 = H->size[0] * H->size[1];
    H->size[0] = b_H->size[0];
    H->size[1] = b_H->size[1];
    emxEnsureCapacity_real_T(H, i0);
    ar = b_H->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      jmax = b_H->size[0];
      for (i1 = 0; i1 < jmax; i1++) {
        H->data[i1 + H->size[0] * i0] = b_H->data[i1 + b_H->size[0] * i0];
      }
    }
  }

  emxFree_real_T(&b_H);
  if ((H->size[0] == 0) || (H->size[1] == 0)) {
    i0 = Q->size[0] * Q->size[1];
    Q->size[0] = H->size[0];
    Q->size[1] = H->size[1];
    emxEnsureCapacity_real_T(Q, i0);
    ar = H->size[0] * H->size[1];
    for (i0 = 0; i0 < ar; i0++) {
      Q->data[i0] = H->data[i0];
    }
  } else {
    invNxN(H, Q);
  }

  emxFree_real_T(&H);
  unnamed_idx_0 = (unsigned int)S->size[0];

  //      [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb);
  i0 = c_C->size[0];
  c_C->size[0] = b_C->size[1];
  emxEnsureCapacity_real_T1(c_C, i0);
  ar = b_C->size[1];
  for (i0 = 0; i0 < ar; i0++) {
    c_C->data[i0] = b_C->data[b_C->size[0] * i0];
  }

  emxFree_real_T(&b_C);
  emxInit_boolean_T(&r2, 1);
  i0 = r2->size[0];
  r2->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_boolean_T(r2, i0);
  ar = (int)unnamed_idx_0;
  for (i0 = 0; i0 < ar; i0++) {
    r2->data[i0] = false;
  }

  mpcqpsolver(Q, c_C, G, S, r2, x, &ajj);
  emxFree_boolean_T(&r2);
  emxFree_real_T(&c_C);
  emxFree_real_T(&S);
  emxFree_real_T(&G);
  emxFree_real_T(&Q);
}

//
// File trailer for solveQP.cpp
//
// [EOF]
//
