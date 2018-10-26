/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveQP.cpp
 *
 * Code generation for function 'solveQP'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "xscal.h"
#include "solveQP_emxutil.h"
#include "mpcqpsolver.h"
#include "inv.h"
#include "eye.h"
#include "mpower.h"

/* Function Definitions */
void solveQP(double P, double M, const double xk[4], const emxArray_real_T *rp,
             double q, double k, double Qf, double Qb, emxArray_real_T *x)
{
  emxArray_real_T *Q;
  int i0;
  int i1;
  int loop_ub;
  double a;
  int i;
  emxArray_real_T *Ap;
  double temp;
  double Ap0[16];
  emxArray_real_T *Bp;
  int iy;
  double b_Ap0[16];
  int i2;
  double c;
  int nmj;
  int coffset;
  static const double b[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.025,
    0.0, 0.9994, 0.0, 0.0, 0.025, 0.0, 0.9994 };

  emxArray_real_T *Bpj;
  emxArray_real_T *Linv;
  emxArray_real_T *b_a;
  double Bpi[8];
  static const double B[8] = { -0.0031, 0.0, -0.2451, 0.0, 0.0, -0.0031, 0.0,
    -0.2451 };

  int jj;
  int m;
  int inner;
  int ix;
  int n;
  int j;
  emxArray_real_T *r0;
  int boffset;
  int aoffset;
  double b_b[8];
  int b_k;
  emxArray_real_T *H;
  emxArray_real_T *y;
  emxArray_real_T *b_y;
  emxArray_real_T *c_a;
  emxArray_real_T *G;
  emxArray_real_T *c_y;
  emxArray_real_T *S;
  emxArray_int8_T *d_a;
  emxArray_int32_T *r1;
  emxArray_real_T *d_y;
  emxArray_real_T *r2;
  boolean_T exitg1;
  unsigned int unnamed_idx_0;
  emxArray_boolean_T *r3;
  emxInit_real_T(&Q, 2);

  /*  input */
  /*  P = 12; */
  /*  M = 5; */
  /*  % rp = repmat([100,100,0,0]', P ,1); */
  /*  % xk = [0,0,0,0]'; */
  /*   */
  /*  q = 0.99; */
  eye(4.0 * P, Q);
  i0 = Q->size[0] * Q->size[1];
  i1 = Q->size[0] * Q->size[1];
  emxEnsureCapacity_real_T(Q, i1);
  loop_ub = i0 - 1;
  for (i0 = 0; i0 <= loop_ub; i0++) {
    Q->data[i0] *= q;
  }

  a = 0.35 * (1.0 - q);

  /*   */
  /*  k = 7; */
  i0 = (int)(k - 1.0);
  for (i = 0; i < i0; i++) {
    i1 = 4 * (1 + i);
    Q->data[i1 + Q->size[0] * i1] = Qf;

    /* 10; */
    Q->data[(i1 + Q->size[0] * (i1 + 1)) + 1] = Qf;

    /* 10; */
  }

  i0 = (int)((P - 1.0) + (1.0 - k));
  for (i = 0; i < i0; i++) {
    temp = k + (double)i;
    i1 = (int)(4.0 * temp + 1.0);
    Q->data[(i1 + Q->size[0] * (i1 - 1)) - 1] = Qb;

    /* 1.15; */
    i1 = (int)(4.0 * temp + 2.0);
    Q->data[(i1 + Q->size[0] * (i1 - 1)) - 1] = Qb;

    /* 1.15; */
  }

  emxInit_real_T(&Ap, 2);

  /*  */
  /*      H = [1 -1; -1 2]; */
  /*      f = [-2; -6]; */
  /*      A = [1 1; -1 2; 2 1]; */
  /*      b = [2; 2; 3]; */
  /*      lb = [0; 0]; */
  /*      [L,p] = chol(H,'lower'); */
  /*      Linv = inv(L); */
  /*      opt = mpcqpsolverOptions; */
  /*      iA0 = false(size(b)); */
  /*      Aeq = []; */
  /*      beq = zeros(0,1); */
  /*  %     [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb); */
  /*      [x,status] = mpcqpsolver(Linv,f,A,b,Aeq,beq,iA0,opt); */
  /*  A = [1    0   0.0498  0.0001;   */
  /*       0    1   -0.0002   0.0495; */
  /*       0.0009    0.0008   0.9895   0.0038; */
  /*       -0.0011    -0.0009   -0.0064   0.9792 ]; */
  /*   */
  /*  B = [0.0003     -0.0006; */
  /*       -0.0001    0.0004; */
  /*       -0.0035    -0.0051; */
  /*       0.0132     -0.0049]; */
  /*  Build Ap */
  i0 = Ap->size[0] * Ap->size[1];
  i1 = (int)(4.0 * P);
  Ap->size[0] = i1;
  Ap->size[1] = 4;
  emxEnsureCapacity_real_T(Ap, i0);
  loop_ub = i1 << 2;
  for (i0 = 0; i0 < loop_ub; i0++) {
    Ap->data[i0] = 0.0;
  }

  b_eye(Ap0);
  i0 = (int)P;
  for (i = 0; i < i0; i++) {
    for (iy = 0; iy < 4; iy++) {
      for (i2 = 0; i2 < 4; i2++) {
        nmj = i2 << 2;
        coffset = iy + nmj;
        b_Ap0[coffset] = 0.0;
        b_Ap0[coffset] = ((Ap0[iy] * b[nmj] + Ap0[iy + 4] * b[1 + nmj]) + Ap0[iy
                          + 8] * b[2 + nmj]) + Ap0[iy + 12] * b[3 + nmj];
      }
    }

    memcpy(&Ap0[0], &b_Ap0[0], sizeof(double) << 4);
    c = ((1.0 + (double)i) - 1.0) * 4.0;
    for (iy = 0; iy < 4; iy++) {
      i2 = iy << 2;
      Ap->data[(int)c + Ap->size[0] * iy] = Ap0[i2];
      Ap->data[((int)c + Ap->size[0] * iy) + 1] = Ap0[1 + i2];
      Ap->data[((int)c + Ap->size[0] * iy) + 2] = Ap0[2 + i2];
      Ap->data[((int)c + Ap->size[0] * iy) + 3] = Ap0[3 + i2];
    }
  }

  emxInit_real_T(&Bp, 2);

  /*  Build Bp */
  iy = Bp->size[0] * Bp->size[1];
  Bp->size[0] = i1;
  i2 = (int)(2.0 * M);
  Bp->size[1] = i2;
  emxEnsureCapacity_real_T(Bp, iy);
  loop_ub = i1 * i2;
  for (iy = 0; iy < loop_ub; iy++) {
    Bp->data[iy] = 0.0;
  }

  emxInit_real_T(&Bpj, 2);
  for (i = 0; i < i0; i++) {
    iy = Bpj->size[0] * Bpj->size[1];
    Bpj->size[0] = 4;
    Bpj->size[1] = i2;
    emxEnsureCapacity_real_T(Bpj, iy);
    loop_ub = i2 << 2;
    for (iy = 0; iy < loop_ub; iy++) {
      Bpj->data[iy] = 0.0;
    }

    if (1.0 + (double)i < M) {
      memcpy(&Bpi[0], &B[0], sizeof(double) << 3);
    } else {
      mpower(b, (1.0 + (double)i) - M, Ap0);
      for (iy = 0; iy < 4; iy++) {
        for (ix = 0; ix < 2; ix++) {
          nmj = ix << 2;
          coffset = iy + nmj;
          Bpi[coffset] = 0.0;
          Bpi[coffset] = ((Ap0[iy] * B[nmj] + Ap0[iy + 4] * B[1 + nmj]) + Ap0[iy
                          + 8] * B[2 + nmj]) + Ap0[iy + 12] * B[3 + nmj];
        }
      }
    }

    temp = 1.0 + (double)i;
    if (M < temp) {
      temp = M;
    }

    iy = (int)((1.0 + (-1.0 - temp)) / -1.0);
    for (j = 0; j < iy; j++) {
      c = ((temp + -(double)j) - 1.0) * 2.0;
      for (ix = 0; ix < 2; ix++) {
        boffset = ix << 2;
        nmj = ((int)(c + (1.0 + (double)ix)) - 1) << 2;
        Bpj->data[nmj] = Bpi[boffset];
        Bpj->data[1 + nmj] = Bpi[1 + boffset];
        Bpj->data[2 + nmj] = Bpi[2 + boffset];
        Bpj->data[3 + nmj] = Bpi[3 + boffset];
      }

      for (ix = 0; ix < 4; ix++) {
        for (boffset = 0; boffset < 2; boffset++) {
          nmj = boffset << 2;
          coffset = ix + nmj;
          b_b[coffset] = 0.0;
          b_b[coffset] = ((b[ix] * Bpi[nmj] + b[ix + 4] * Bpi[1 + nmj]) + b[ix +
                          8] * Bpi[2 + nmj]) + b[ix + 12] * Bpi[3 + nmj];
        }
      }

      memcpy(&Bpi[0], &b_b[0], sizeof(double) << 3);
    }

    c = ((1.0 + (double)i) - 1.0) * 4.0;
    coffset = (int)(c + 1.0) - 1;
    nmj = (int)c;
    boffset = nmj + 1;
    aoffset = nmj + 2;
    nmj += 3;
    loop_ub = Bpj->size[1];
    for (iy = 0; iy < loop_ub; iy++) {
      ix = iy << 2;
      Bp->data[coffset + Bp->size[0] * iy] = Bpj->data[ix];
      Bp->data[boffset + Bp->size[0] * iy] = Bpj->data[1 + ix];
      Bp->data[aoffset + Bp->size[0] * iy] = Bpj->data[2 + ix];
      Bp->data[nmj + Bp->size[0] * iy] = Bpj->data[3 + ix];
    }

    /*                  disp(Bp); */
  }

  emxInit_real_T(&Linv, 2);
  emxInit_real_T(&b_a, 2);
  i0 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = Bp->size[1];
  b_a->size[1] = Bp->size[0];
  emxEnsureCapacity_real_T(b_a, i0);
  loop_ub = Bp->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    jj = Bp->size[1];
    for (iy = 0; iy < jj; iy++) {
      b_a->data[iy + b_a->size[0] * i0] = Bp->data[i0 + Bp->size[0] * iy];
    }
  }

  if ((b_a->size[1] == 1) || (Q->size[0] == 1)) {
    i0 = Linv->size[0] * Linv->size[1];
    Linv->size[0] = b_a->size[0];
    Linv->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(Linv, i0);
    loop_ub = b_a->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      jj = Q->size[1];
      for (iy = 0; iy < jj; iy++) {
        Linv->data[i0 + Linv->size[0] * iy] = 0.0;
        boffset = b_a->size[1];
        for (ix = 0; ix < boffset; ix++) {
          Linv->data[i0 + Linv->size[0] * iy] += b_a->data[i0 + b_a->size[0] *
            ix] * Q->data[ix + Q->size[0] * iy];
        }
      }
    }
  } else {
    m = b_a->size[0];
    inner = b_a->size[1];
    n = Q->size[1];
    i0 = Linv->size[0] * Linv->size[1];
    Linv->size[0] = b_a->size[0];
    Linv->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(Linv, i0);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        Linv->data[coffset + i] = 0.0;
      }

      for (b_k = 0; b_k < inner; b_k++) {
        aoffset = b_k * m;
        temp = Q->data[boffset + b_k];
        for (i = 0; i < m; i++) {
          i0 = coffset + i;
          Linv->data[i0] += temp * b_a->data[aoffset + i];
        }
      }
    }
  }

  emxInit_real_T(&r0, 2);
  if ((Linv->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = Linv->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i0);
    loop_ub = Linv->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      jj = Bp->size[1];
      for (iy = 0; iy < jj; iy++) {
        r0->data[i0 + r0->size[0] * iy] = 0.0;
        boffset = Linv->size[1];
        for (ix = 0; ix < boffset; ix++) {
          r0->data[i0 + r0->size[0] * iy] += Linv->data[i0 + Linv->size[0] * ix]
            * Bp->data[ix + Bp->size[0] * iy];
        }
      }
    }
  } else {
    m = Linv->size[0];
    inner = Linv->size[1];
    n = Bp->size[1];
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = Linv->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i0);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        r0->data[coffset + i] = 0.0;
      }

      for (b_k = 0; b_k < inner; b_k++) {
        aoffset = b_k * m;
        temp = Bp->data[boffset + b_k];
        for (i = 0; i < m; i++) {
          i0 = coffset + i;
          r0->data[i0] += temp * Linv->data[aoffset + i];
        }
      }
    }
  }

  emxInit_real_T(&H, 2);
  eye(2.0 * M, H);
  i0 = H->size[0] * H->size[1];
  iy = H->size[0] * H->size[1];
  emxEnsureCapacity_real_T(H, iy);
  loop_ub = i0 - 1;
  for (i0 = 0; i0 <= loop_ub; i0++) {
    H->data[i0] = a * H->data[i0] + r0->data[i0];
  }

  i0 = Bpj->size[0] * Bpj->size[1];
  Bpj->size[0] = 4;
  Bpj->size[1] = Ap->size[0];
  emxEnsureCapacity_real_T(Bpj, i0);
  loop_ub = Ap->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    iy = i0 << 2;
    Bpj->data[iy] = Ap->data[i0];
    Bpj->data[1 + iy] = Ap->data[i0 + Ap->size[0]];
    Bpj->data[2 + iy] = Ap->data[i0 + (Ap->size[0] << 1)];
    Bpj->data[3 + iy] = Ap->data[i0 + Ap->size[0] * 3];
  }

  emxInit_real_T(&y, 2);
  n = Bpj->size[1];
  i0 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = Bpj->size[1];
  emxEnsureCapacity_real_T(y, i0);
  for (j = 0; j < n; j++) {
    boffset = j << 2;
    y->data[j] = 0.0;
    y->data[j] += Bpj->data[boffset] * xk[0];
    y->data[j] += Bpj->data[boffset + 1] * xk[1];
    y->data[j] += Bpj->data[boffset + 2] * xk[2];
    y->data[j] += Bpj->data[boffset + 3] * xk[3];
  }

  emxFree_real_T(&Bpj);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&c_a, 2);
  i0 = c_a->size[0] * c_a->size[1];
  c_a->size[0] = 1;
  c_a->size[1] = y->size[1];
  emxEnsureCapacity_real_T(c_a, i0);
  loop_ub = y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_a->data[i0] = y->data[i0] - rp->data[i0];
  }

  if ((c_a->size[1] == 1) || (Q->size[0] == 1)) {
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(b_y, i0);
    loop_ub = Q->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_y->data[i0] = 0.0;
      jj = c_a->size[1];
      for (iy = 0; iy < jj; iy++) {
        b_y->data[i0] += c_a->data[iy] * Q->data[iy + Q->size[0] * i0];
      }
    }
  } else {
    inner = c_a->size[1];
    n = Q->size[1];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = 1;
    b_y->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(b_y, i0);
    for (j = 0; j < n; j++) {
      boffset = j * inner;
      b_y->data[j] = 0.0;
      for (b_k = 0; b_k < inner; b_k++) {
        b_y->data[j] += Q->data[boffset + b_k] * c_a->data[b_k];
      }
    }
  }

  emxFree_real_T(&c_a);
  if ((b_y->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(y, i0);
    loop_ub = Bp->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      y->data[i0] = 0.0;
      jj = b_y->size[1];
      for (iy = 0; iy < jj; iy++) {
        y->data[i0] += b_y->data[iy] * Bp->data[iy + Bp->size[0] * i0];
      }
    }
  } else {
    inner = b_y->size[1];
    n = Bp->size[1];
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(y, i0);
    for (j = 0; j < n; j++) {
      boffset = j * inner;
      y->data[j] = 0.0;
      for (b_k = 0; b_k < inner; b_k++) {
        y->data[j] += Bp->data[boffset + b_k] * b_y->data[b_k];
      }
    }
  }

  emxFree_real_T(&b_y);
  i0 = Q->size[0] * Q->size[1];
  loop_ub = (int)(2.0 * P);
  Q->size[0] = loop_ub;
  Q->size[1] = i1;
  emxEnsureCapacity_real_T(Q, i0);
  jj = loop_ub * i1;
  for (i0 = 0; i0 < jj; i0++) {
    Q->data[i0] = 0.0;
  }

  i0 = (int)((P - 1.0) + 1.0);
  for (i = 0; i < i0; i++) {
    Q->data[(int)((unsigned int)i << 1) + Q->size[0] * (4 * i + 2)] = 1.0;
    Q->data[(2 * i + Q->size[0] * (4 * i + 3)) + 1] = 1.0;
  }

  emxInit_real_T(&G, 2);
  i0 = G->size[0] * G->size[1];
  jj = (int)(4.0 * P + 4.0 * M);
  G->size[0] = jj;
  G->size[1] = i2;
  emxEnsureCapacity_real_T(G, i0);
  boffset = jj * i2;
  for (i0 = 0; i0 < boffset; i0++) {
    G->data[i0] = 0.0;
  }

  c_eye(2.0 * M, 2.0 * M, Linv);
  boffset = Linv->size[1];
  for (i0 = 0; i0 < boffset; i0++) {
    nmj = Linv->size[0];
    for (i1 = 0; i1 < nmj; i1++) {
      G->data[i1 + G->size[0] * i0] = Linv->data[i1 + Linv->size[0] * i0];
    }
  }

  c = 2.0 * M + 1.0;
  if (c > 4.0 * M) {
    i0 = 0;
  } else {
    i0 = (int)c - 1;
  }

  boffset = Linv->size[1];
  for (i1 = 0; i1 < boffset; i1++) {
    nmj = Linv->size[0];
    for (iy = 0; iy < nmj; iy++) {
      G->data[(i0 + iy) + G->size[0] * i1] = -Linv->data[iy + Linv->size[0] * i1];
    }
  }

  emxInit_real_T(&c_y, 1);
  c = 4.0 * M;
  i0 = c_y->size[0];
  boffset = (int)std::floor(2.0 * P - 1.0);
  c_y->size[0] = boffset + 1;
  emxEnsureCapacity_real_T(c_y, i0);
  for (i0 = 0; i0 <= boffset; i0++) {
    c_y->data[i0] = c + (1.0 + (double)i0);
  }

  if ((Q->size[1] == 1) || (Bp->size[0] == 1)) {
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = Q->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i0);
    boffset = Q->size[0];
    for (i0 = 0; i0 < boffset; i0++) {
      nmj = Bp->size[1];
      for (i1 = 0; i1 < nmj; i1++) {
        r0->data[i0 + r0->size[0] * i1] = 0.0;
        coffset = Q->size[1];
        for (iy = 0; iy < coffset; iy++) {
          r0->data[i0 + r0->size[0] * i1] += Q->data[i0 + Q->size[0] * iy] *
            Bp->data[iy + Bp->size[0] * i1];
        }
      }
    }
  } else {
    m = Q->size[0];
    inner = Q->size[1];
    n = Bp->size[1];
    i0 = r0->size[0] * r0->size[1];
    r0->size[0] = Q->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i0);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        r0->data[coffset + i] = 0.0;
      }

      for (b_k = 0; b_k < inner; b_k++) {
        aoffset = b_k * m;
        temp = Bp->data[boffset + b_k];
        for (i = 0; i < m; i++) {
          i0 = coffset + i;
          r0->data[i0] += temp * (double)(signed char)Q->data[aoffset + i];
        }
      }
    }
  }

  boffset = r0->size[1];
  for (i0 = 0; i0 < boffset; i0++) {
    nmj = r0->size[0];
    for (i1 = 0; i1 < nmj; i1++) {
      G->data[((int)c_y->data[i1] + G->size[0] * i0) - 1] = r0->data[i1 +
        r0->size[0] * i0];
    }
  }

  c = (2.0 * P + 4.0 * M) + 1.0;
  if (c > 4.0 * P + 4.0 * M) {
    i0 = 0;
  } else {
    i0 = (int)c - 1;
  }

  i1 = b_a->size[0] * b_a->size[1];
  b_a->size[0] = Q->size[0];
  b_a->size[1] = Q->size[1];
  emxEnsureCapacity_real_T(b_a, i1);
  boffset = Q->size[0] * Q->size[1];
  for (i1 = 0; i1 < boffset; i1++) {
    b_a->data[i1] = -Q->data[i1];
  }

  if ((b_a->size[1] == 1) || (Bp->size[0] == 1)) {
    i1 = r0->size[0] * r0->size[1];
    r0->size[0] = b_a->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i1);
    boffset = b_a->size[0];
    for (i1 = 0; i1 < boffset; i1++) {
      nmj = Bp->size[1];
      for (iy = 0; iy < nmj; iy++) {
        r0->data[i1 + r0->size[0] * iy] = 0.0;
        coffset = b_a->size[1];
        for (i2 = 0; i2 < coffset; i2++) {
          r0->data[i1 + r0->size[0] * iy] += b_a->data[i1 + b_a->size[0] * i2] *
            Bp->data[i2 + Bp->size[0] * iy];
        }
      }
    }
  } else {
    m = b_a->size[0];
    inner = b_a->size[1];
    n = Bp->size[1];
    i1 = r0->size[0] * r0->size[1];
    r0->size[0] = b_a->size[0];
    r0->size[1] = Bp->size[1];
    emxEnsureCapacity_real_T(r0, i1);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        r0->data[coffset + i] = 0.0;
      }

      for (b_k = 0; b_k < inner; b_k++) {
        aoffset = b_k * m;
        temp = Bp->data[boffset + b_k];
        for (i = 0; i < m; i++) {
          i1 = coffset + i;
          r0->data[i1] += temp * (double)(signed char)b_a->data[aoffset + i];
        }
      }
    }
  }

  emxFree_real_T(&b_a);
  emxFree_real_T(&Bp);
  boffset = r0->size[1];
  for (i1 = 0; i1 < boffset; i1++) {
    nmj = r0->size[0];
    for (iy = 0; iy < nmj; iy++) {
      G->data[(i0 + iy) + G->size[0] * i1] = r0->data[iy + r0->size[0] * i1];
    }
  }

  emxFree_real_T(&r0);
  i0 = G->size[0] * G->size[1];
  i1 = G->size[0] * G->size[1];
  emxEnsureCapacity_real_T(G, i1);
  boffset = i0 - 1;
  for (i0 = 0; i0 <= boffset; i0++) {
    G->data[i0] = -G->data[i0];
  }

  emxInit_real_T(&S, 1);
  i0 = S->size[0];
  S->size[0] = jj;
  emxEnsureCapacity_real_T(S, i0);
  for (i0 = 0; i0 < jj; i0++) {
    S->data[i0] = 0.0;
  }

  c = 4.0 * M;
  if (1.0 > c) {
    jj = 0;
  } else {
    jj = (int)c;
  }

  emxInit_int8_T(&d_a, 1);
  i0 = d_a->size[0];
  boffset = (int)(4.0 * M);
  d_a->size[0] = boffset;
  emxEnsureCapacity_int8_T(d_a, i0);
  for (i0 = 0; i0 < boffset; i0++) {
    d_a->data[i0] = 1;
  }

  emxInit_int32_T(&r1, 2);
  i0 = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = jj;
  emxEnsureCapacity_int32_T(r1, i0);
  for (i0 = 0; i0 < jj; i0++) {
    r1->data[i0] = i0;
  }

  jj = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < jj; i0++) {
    S->data[r1->data[i0]] = (signed char)(d_a->data[i0] * 20);
  }

  i0 = d_a->size[0];
  d_a->size[0] = loop_ub;
  emxEnsureCapacity_int8_T(d_a, i0);
  for (i0 = 0; i0 < loop_ub; i0++) {
    d_a->data[i0] = 1;
  }

  emxInit_real_T(&d_y, 2);
  if ((Q->size[1] == 1) || (Ap->size[0] == 1)) {
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = Q->size[0];
    d_y->size[1] = 4;
    emxEnsureCapacity_real_T(d_y, i0);
    jj = Q->size[0];
    for (i0 = 0; i0 < jj; i0++) {
      d_y->data[i0] = 0.0;
      boffset = Q->size[1];
      for (i1 = 0; i1 < boffset; i1++) {
        d_y->data[i0] += Q->data[i0 + Q->size[0] * i1] * Ap->data[i1];
      }

      d_y->data[i0 + d_y->size[0]] = 0.0;
      boffset = Q->size[1];
      for (i1 = 0; i1 < boffset; i1++) {
        d_y->data[i0 + d_y->size[0]] += Q->data[i0 + Q->size[0] * i1] * Ap->
          data[i1 + Ap->size[0]];
      }

      d_y->data[i0 + (d_y->size[0] << 1)] = 0.0;
      boffset = Q->size[1];
      for (i1 = 0; i1 < boffset; i1++) {
        d_y->data[i0 + (d_y->size[0] << 1)] += Q->data[i0 + Q->size[0] * i1] *
          Ap->data[i1 + (Ap->size[0] << 1)];
      }

      d_y->data[i0 + d_y->size[0] * 3] = 0.0;
      boffset = Q->size[1];
      for (i1 = 0; i1 < boffset; i1++) {
        d_y->data[i0 + d_y->size[0] * 3] += Q->data[i0 + Q->size[0] * i1] *
          Ap->data[i1 + Ap->size[0] * 3];
      }
    }
  } else {
    m = Q->size[0];
    inner = Q->size[1];
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = Q->size[0];
    d_y->size[1] = 4;
    emxEnsureCapacity_real_T(d_y, i0);
    for (i = 0; i < m; i++) {
      d_y->data[i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      nmj = b_k * m;
      temp = Ap->data[b_k];
      for (i = 0; i < m; i++) {
        d_y->data[i] += temp * (double)(signed char)Q->data[nmj + i];
      }
    }

    for (i = 0; i < m; i++) {
      d_y->data[m + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[inner + b_k];
      for (i = 0; i < m; i++) {
        i1 = m + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }

    coffset = m << 1;
    boffset = inner << 1;
    for (i = 0; i < m; i++) {
      d_y->data[coffset + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[boffset + b_k];
      for (i = 0; i < m; i++) {
        i1 = coffset + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }

    coffset = 3 * m;
    boffset = 3 * inner;
    for (i = 0; i < m; i++) {
      d_y->data[coffset + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[boffset + b_k];
      for (i = 0; i < m; i++) {
        i1 = coffset + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }
  }

  emxInit_real_T(&r2, 1);
  m = d_y->size[0];
  i0 = r2->size[0];
  r2->size[0] = d_y->size[0];
  emxEnsureCapacity_real_T(r2, i0);
  for (i = 0; i < m; i++) {
    r2->data[i] = 0.0;
  }

  for (i = 0; i < m; i++) {
    r2->data[i] += xk[0] * d_y->data[i];
  }

  for (i = 0; i < m; i++) {
    r2->data[i] += xk[1] * d_y->data[m + i];
  }

  aoffset = m << 1;
  for (i = 0; i < m; i++) {
    r2->data[i] += xk[2] * d_y->data[aoffset + i];
  }

  aoffset = 3 * m;
  for (i = 0; i < m; i++) {
    r2->data[i] += xk[3] * d_y->data[aoffset + i];
  }

  i0 = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  r1->size[1] = c_y->size[0];
  emxEnsureCapacity_int32_T(r1, i0);
  jj = c_y->size[0];
  for (i0 = 0; i0 < jj; i0++) {
    r1->data[i0] = (int)c_y->data[i0];
  }

  jj = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < jj; i0++) {
    S->data[r1->data[i0] - 1] = (double)d_a->data[i0] * 17.0 - r2->data[i0];
  }

  emxFree_real_T(&r2);
  i0 = d_a->size[0];
  d_a->size[0] = loop_ub;
  emxEnsureCapacity_int8_T(d_a, i0);
  for (i0 = 0; i0 < loop_ub; i0++) {
    d_a->data[i0] = 1;
  }

  if ((Q->size[1] == 1) || (Ap->size[0] == 1)) {
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = Q->size[0];
    d_y->size[1] = 4;
    emxEnsureCapacity_real_T(d_y, i0);
    loop_ub = Q->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      d_y->data[i0] = 0.0;
      jj = Q->size[1];
      for (i1 = 0; i1 < jj; i1++) {
        d_y->data[i0] += Q->data[i0 + Q->size[0] * i1] * Ap->data[i1];
      }

      d_y->data[i0 + d_y->size[0]] = 0.0;
      jj = Q->size[1];
      for (i1 = 0; i1 < jj; i1++) {
        d_y->data[i0 + d_y->size[0]] += Q->data[i0 + Q->size[0] * i1] * Ap->
          data[i1 + Ap->size[0]];
      }

      d_y->data[i0 + (d_y->size[0] << 1)] = 0.0;
      jj = Q->size[1];
      for (i1 = 0; i1 < jj; i1++) {
        d_y->data[i0 + (d_y->size[0] << 1)] += Q->data[i0 + Q->size[0] * i1] *
          Ap->data[i1 + (Ap->size[0] << 1)];
      }

      d_y->data[i0 + d_y->size[0] * 3] = 0.0;
      jj = Q->size[1];
      for (i1 = 0; i1 < jj; i1++) {
        d_y->data[i0 + d_y->size[0] * 3] += Q->data[i0 + Q->size[0] * i1] *
          Ap->data[i1 + Ap->size[0] * 3];
      }
    }
  } else {
    m = Q->size[0];
    inner = Q->size[1];
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = Q->size[0];
    d_y->size[1] = 4;
    emxEnsureCapacity_real_T(d_y, i0);
    for (i = 0; i < m; i++) {
      d_y->data[i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      nmj = b_k * m;
      temp = Ap->data[b_k];
      for (i = 0; i < m; i++) {
        d_y->data[i] += temp * (double)(signed char)Q->data[nmj + i];
      }
    }

    for (i = 0; i < m; i++) {
      d_y->data[m + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[inner + b_k];
      for (i = 0; i < m; i++) {
        i1 = m + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }

    coffset = m << 1;
    boffset = inner << 1;
    for (i = 0; i < m; i++) {
      d_y->data[coffset + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[boffset + b_k];
      for (i = 0; i < m; i++) {
        i1 = coffset + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }

    coffset = 3 * m;
    boffset = 3 * inner;
    for (i = 0; i < m; i++) {
      d_y->data[coffset + i] = 0.0;
    }

    for (b_k = 0; b_k < inner; b_k++) {
      i0 = b_k * m;
      temp = Ap->data[boffset + b_k];
      for (i = 0; i < m; i++) {
        i1 = coffset + i;
        d_y->data[i1] += temp * (double)(signed char)Q->data[i0 + i];
      }
    }
  }

  emxFree_real_T(&Ap);
  m = d_y->size[0];
  i0 = c_y->size[0];
  c_y->size[0] = d_y->size[0];
  emxEnsureCapacity_real_T(c_y, i0);
  for (i = 0; i < m; i++) {
    c_y->data[i] = 0.0;
  }

  for (i = 0; i < m; i++) {
    c_y->data[i] += xk[0] * d_y->data[i];
  }

  for (i = 0; i < m; i++) {
    c_y->data[i] += xk[1] * d_y->data[m + i];
  }

  aoffset = m << 1;
  for (i = 0; i < m; i++) {
    c_y->data[i] += xk[2] * d_y->data[aoffset + i];
  }

  aoffset = 3 * m;
  for (i = 0; i < m; i++) {
    c_y->data[i] += xk[3] * d_y->data[aoffset + i];
  }

  emxFree_real_T(&d_y);
  c = (4.0 * M + 1.0) + 2.0 * P;
  temp = 4.0 * M + 4.0 * P;
  if (c > temp) {
    i0 = 0;
    i1 = 0;
  } else {
    i0 = (int)c - 1;
    i1 = (int)temp;
  }

  iy = r1->size[0] * r1->size[1];
  r1->size[0] = 1;
  loop_ub = i1 - i0;
  r1->size[1] = loop_ub;
  emxEnsureCapacity_int32_T(r1, iy);
  for (i1 = 0; i1 < loop_ub; i1++) {
    r1->data[i1] = i0 + i1;
  }

  loop_ub = r1->size[0] * r1->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    S->data[r1->data[i0]] = (double)d_a->data[i0] * 17.0 + c_y->data[i0];
  }

  emxFree_int8_T(&d_a);
  emxFree_int32_T(&r1);
  i0 = S->size[0];
  emxEnsureCapacity_real_T(S, i0);
  loop_ub = S->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    S->data[i0] = -S->data[i0];
  }

  /*  G = eye(2*M, 2*M); */
  /*  G = [G ; -eye(2*M, 2*M)]; */
  /*  G = -G; */
  /*  S = ones(4*M,1)*20; */
  /*  S = -S; */
  /*  G = Cv*Bp; */
  /*  G = [G ; -Cv*Bp]; */
  /*   */
  /*  S = ones(2*P,1)*17 - Cv*Ap*xk; */
  /*  S = [S; ones(2*P,1)*17 + Cv*Ap*xk]; */
  /*  ub = ones(2*M,1)*20; */
  /*  lb = ones(2*M,1)*(-20); */
  n = H->size[1];
  if (H->size[1] != 0) {
    m = H->size[0];
    inner = 0;
    if (H->size[0] != 0) {
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j <= m - 1)) {
        jj = j + j * n;
        temp = 0.0;
        if (j >= 1) {
          ix = j;
          iy = j;
          for (b_k = 0; b_k < j; b_k++) {
            temp += H->data[ix] * H->data[iy];
            ix += n;
            iy += n;
          }
        }

        temp = H->data[jj] - temp;
        if (temp > 0.0) {
          temp = std::sqrt(temp);
          H->data[jj] = temp;
          if (j + 1 < m) {
            nmj = (m - j) - 1;
            coffset = j + 2;
            if ((nmj == 0) || (j == 0)) {
            } else {
              ix = j;
              i0 = (j + n * (j - 1)) + 2;
              for (boffset = coffset; n < 0 ? boffset >= i0 : boffset <= i0;
                   boffset += n) {
                c = -H->data[ix];
                iy = jj + 1;
                i1 = (boffset + nmj) - 1;
                for (aoffset = boffset; aoffset <= i1; aoffset++) {
                  H->data[iy] += H->data[aoffset - 1] * c;
                  iy++;
                }

                ix += n;
              }
            }

            xscal(nmj, 1.0 / temp, H, jj + 2);
          }

          j++;
        } else {
          H->data[jj] = temp;
          inner = j + 1;
          exitg1 = true;
        }
      }
    }

    if (inner == 0) {
      nmj = n;
    } else {
      nmj = inner - 1;
    }

    for (j = 2; j <= nmj; j++) {
      for (i = 0; i <= j - 2; i++) {
        H->data[i + H->size[0] * (j - 1)] = 0.0;
      }
    }

    if (1 > nmj) {
      loop_ub = 0;
      jj = 0;
    } else {
      loop_ub = nmj;
      jj = nmj;
    }

    i0 = Q->size[0] * Q->size[1];
    Q->size[0] = loop_ub;
    Q->size[1] = jj;
    emxEnsureCapacity_real_T(Q, i0);
    for (i0 = 0; i0 < jj; i0++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        Q->data[i1 + Q->size[0] * i0] = H->data[i1 + H->size[0] * i0];
      }
    }

    i0 = H->size[0] * H->size[1];
    H->size[0] = Q->size[0];
    H->size[1] = Q->size[1];
    emxEnsureCapacity_real_T(H, i0);
    loop_ub = Q->size[0] * Q->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      H->data[i0] = Q->data[i0];
    }
  }

  emxFree_real_T(&Q);
  if ((H->size[0] == 0) || (H->size[1] == 0)) {
    i0 = Linv->size[0] * Linv->size[1];
    Linv->size[0] = H->size[0];
    Linv->size[1] = H->size[1];
    emxEnsureCapacity_real_T(Linv, i0);
    loop_ub = H->size[0] * H->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      Linv->data[i0] = H->data[i0];
    }
  } else {
    invNxN(H, Linv);
  }

  emxFree_real_T(&H);
  unnamed_idx_0 = (unsigned int)S->size[0];

  /*      [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb); */
  i0 = c_y->size[0];
  c_y->size[0] = y->size[1];
  emxEnsureCapacity_real_T(c_y, i0);
  loop_ub = y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_y->data[i0] = y->data[i0];
  }

  emxFree_real_T(&y);
  emxInit_boolean_T(&r3, 1);
  i0 = r3->size[0];
  r3->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity_boolean_T(r3, i0);
  loop_ub = (int)unnamed_idx_0;
  for (i0 = 0; i0 < loop_ub; i0++) {
    r3->data[i0] = false;
  }

  mpcqpsolver(Linv, c_y, G, S, r3, x, &temp);
  emxFree_boolean_T(&r3);
  emxFree_real_T(&c_y);
  emxFree_real_T(&Linv);
  emxFree_real_T(&S);
  emxFree_real_T(&G);
}

/* End of code generation (solveQP.cpp) */
