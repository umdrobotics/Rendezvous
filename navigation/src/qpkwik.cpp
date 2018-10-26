/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qpkwik.cpp
 *
 * Code generation for function 'qpkwik'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "qpkwik.h"
#include "solveQP_emxutil.h"
#include "qr.h"
#include "abs.h"
#include "norm.h"

/* Function Declarations */
static void DropConstraint(short kDrop, emxArray_int16_T *iA, short *nA, short
  iC_data[]);
static double KWIKfactor(const emxArray_real_T *Ac, const short iC_data[], short
  nA, const emxArray_real_T *Linv, emxArray_real_T *RLinv, emxArray_real_T *D,
  emxArray_real_T *H, short n);
static void Unconstrained(const emxArray_real_T *Hinv, const emxArray_real_T *f,
  emxArray_real_T *x, short n);

/* Function Definitions */
static void DropConstraint(short kDrop, emxArray_int16_T *iA, short *nA, short
  iC_data[])
{
  int i22;
  short i23;
  short i;
  iA->data[iC_data[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    i22 = *nA - 1;
    if (i22 < -32768) {
      i22 = -32768;
    }

    i23 = (short)i22;
    for (i = kDrop; i <= i23; i++) {
      iC_data[i - 1] = iC_data[i];
    }
  }

  iC_data[*nA - 1] = 0;
  i22 = *nA - 1;
  if (i22 < -32768) {
    i22 = -32768;
  }

  *nA = (short)i22;
}

static double KWIKfactor(const emxArray_real_T *Ac, const short iC_data[], short
  nA, const emxArray_real_T *Linv, emxArray_real_T *RLinv, emxArray_real_T *D,
  emxArray_real_T *H, short n)
{
  double Status;
  emxArray_real_T *TL;
  int i15;
  int loop_ub;
  int b_loop_ub;
  int i16;
  emxArray_real_T *r4;
  emxArray_real_T *b;
  int i;
  emxArray_real_T *QQ;
  emxArray_real_T *RR;
  emxArray_real_T *a;
  int exitg1;
  int m;
  int inner;
  int aoffset;
  short j;
  short k;
  short b_i;
  short i17;
  boolean_T guard1 = false;
  double b_a;
  emxInit_real_T(&TL, 2);
  i15 = TL->size[0] * TL->size[1];
  TL->size[0] = Linv->size[0];
  TL->size[1] = Linv->size[1];
  emxEnsureCapacity_real_T(TL, i15);
  Status = 1.0;
  loop_ub = RLinv->size[0];
  b_loop_ub = RLinv->size[1];
  for (i15 = 0; i15 < b_loop_ub; i15++) {
    for (i16 = 0; i16 < loop_ub; i16++) {
      RLinv->data[i16 + RLinv->size[0] * i15] = 0.0;
    }
  }

  i15 = nA;
  emxInit_real_T(&r4, 1);
  emxInit_real_T(&b, 1);
  for (i = 0; i < i15; i++) {
    loop_ub = Ac->size[1];
    i16 = b->size[0];
    b->size[0] = loop_ub;
    emxEnsureCapacity_real_T(b, i16);
    for (i16 = 0; i16 < loop_ub; i16++) {
      b->data[i16] = Ac->data[(iC_data[i] + Ac->size[0] * i16) - 1];
    }

    if ((Linv->size[1] == 1) || (b->size[0] == 1)) {
      i16 = r4->size[0];
      r4->size[0] = Linv->size[0];
      emxEnsureCapacity_real_T(r4, i16);
      loop_ub = Linv->size[0];
      for (i16 = 0; i16 < loop_ub; i16++) {
        r4->data[i16] = 0.0;
        b_loop_ub = Linv->size[1];
        for (aoffset = 0; aoffset < b_loop_ub; aoffset++) {
          r4->data[i16] += Linv->data[i16 + Linv->size[0] * aoffset] * b->
            data[aoffset];
        }
      }
    } else {
      m = Linv->size[0];
      inner = Linv->size[1];
      i16 = r4->size[0];
      r4->size[0] = Linv->size[0];
      emxEnsureCapacity_real_T(r4, i16);
      for (b_loop_ub = 0; b_loop_ub < m; b_loop_ub++) {
        r4->data[b_loop_ub] = 0.0;
      }

      for (loop_ub = 0; loop_ub < inner; loop_ub++) {
        aoffset = loop_ub * m;
        for (b_loop_ub = 0; b_loop_ub < m; b_loop_ub++) {
          r4->data[b_loop_ub] += b->data[loop_ub] * Linv->data[aoffset +
            b_loop_ub];
        }
      }
    }

    loop_ub = r4->size[0];
    for (i16 = 0; i16 < loop_ub; i16++) {
      RLinv->data[i16 + RLinv->size[0] * i] = r4->data[i16];
    }
  }

  emxFree_real_T(&r4);
  emxInit_real_T(&QQ, 2);
  emxInit_real_T(&RR, 2);
  qr(RLinv, QQ, RR);
  i = 0;
  emxInit_real_T(&a, 2);
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (std::abs(RR->data[i + RR->size[0] * i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      i15 = n;
      for (i = 0; i < i15; i++) {
        i16 = n;
        for (b_loop_ub = 0; b_loop_ub < i16; b_loop_ub++) {
          loop_ub = Linv->size[0];
          aoffset = a->size[0] * a->size[1];
          a->size[0] = 1;
          a->size[1] = loop_ub;
          emxEnsureCapacity_real_T(a, aoffset);
          for (aoffset = 0; aoffset < loop_ub; aoffset++) {
            a->data[aoffset] = Linv->data[aoffset + Linv->size[0] * i];
          }

          loop_ub = QQ->size[0];
          aoffset = b->size[0];
          b->size[0] = loop_ub;
          emxEnsureCapacity_real_T(b, aoffset);
          for (aoffset = 0; aoffset < loop_ub; aoffset++) {
            b->data[aoffset] = QQ->data[aoffset + QQ->size[0] * b_loop_ub];
          }

          guard1 = false;
          if (a->size[1] == 1) {
            guard1 = true;
          } else {
            aoffset = QQ->size[0];
            if (aoffset == 1) {
              guard1 = true;
            } else {
              b_a = 0.0;
              loop_ub = a->size[1];
              for (aoffset = 0; aoffset < loop_ub; aoffset++) {
                b_a += a->data[aoffset] * b->data[aoffset];
              }

              TL->data[i + TL->size[0] * b_loop_ub] = b_a;
            }
          }

          if (guard1) {
            b_a = 0.0;
            loop_ub = a->size[1];
            for (aoffset = 0; aoffset < loop_ub; aoffset++) {
              b_a += a->data[aoffset] * b->data[aoffset];
            }

            TL->data[i + TL->size[0] * b_loop_ub] = b_a;
          }
        }
      }

      loop_ub = RLinv->size[0];
      b_loop_ub = RLinv->size[1];
      for (i15 = 0; i15 < b_loop_ub; i15++) {
        for (i16 = 0; i16 < loop_ub; i16++) {
          RLinv->data[i16 + RLinv->size[0] * i15] = 0.0;
        }
      }

      for (j = nA; j >= 1; j--) {
        i15 = j - 1;
        RLinv->data[(j + RLinv->size[0] * i15) - 1] = 1.0;
        for (k = j; k <= nA; k++) {
          RLinv->data[(j + RLinv->size[0] * (k - 1)) - 1] /= RR->data[(j +
            RR->size[0] * i15) - 1];
        }

        if (j > 1) {
          i15 = j;
          for (i = 0; i <= i15 - 2; i++) {
            for (k = j; k <= nA; k++) {
              RLinv->data[i + RLinv->size[0] * (k - 1)] -= RR->data[i + RR->
                size[0] * (j - 1)] * RLinv->data[(j + RLinv->size[0] * (k - 1))
                - 1];
            }
          }
        }
      }

      i15 = n;
      for (i = 0; i < i15; i++) {
        b_i = (short)(1 + i);
        for (j = b_i; j <= n; j++) {
          H->data[i + H->size[0] * (j - 1)] = 0.0;
          i16 = nA + 1;
          if (i16 > 32767) {
            i16 = 32767;
          }

          i17 = (short)i16;
          for (k = i17; k <= n; k++) {
            H->data[i + H->size[0] * (j - 1)] -= TL->data[i + TL->size[0] * (k -
              1)] * TL->data[(j + TL->size[0] * (k - 1)) - 1];
          }

          H->data[(j + H->size[0] * i) - 1] = H->data[i + H->size[0] * (j - 1)];
        }
      }

      i15 = nA;
      for (b_loop_ub = 0; b_loop_ub < i15; b_loop_ub++) {
        j = (short)(1 + b_loop_ub);
        i16 = n;
        for (i = 0; i < i16; i++) {
          D->data[i + D->size[0] * b_loop_ub] = 0.0;
          for (k = j; k <= nA; k++) {
            aoffset = k - 1;
            D->data[i + D->size[0] * b_loop_ub] += TL->data[i + TL->size[0] *
              aoffset] * RLinv->data[b_loop_ub + RLinv->size[0] * aoffset];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&a);
  emxFree_real_T(&b);
  emxFree_real_T(&RR);
  emxFree_real_T(&QQ);
  emxFree_real_T(&TL);
  return Status;
}

static void Unconstrained(const emxArray_real_T *Hinv, const emxArray_real_T *f,
  emxArray_real_T *x, short n)
{
  int i13;
  emxArray_real_T *a;
  int i;
  int loop_ub;
  int i14;
  double b_a;
  i13 = n;
  emxInit_real_T(&a, 2);
  for (i = 0; i < i13; i++) {
    loop_ub = Hinv->size[1];
    i14 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = loop_ub;
    emxEnsureCapacity_real_T(a, i14);
    for (i14 = 0; i14 < loop_ub; i14++) {
      a->data[i14] = -Hinv->data[i + Hinv->size[0] * i14];
    }

    if ((a->size[1] == 1) || (f->size[0] == 1)) {
      b_a = 0.0;
      loop_ub = a->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        b_a += a->data[i14] * f->data[i14];
      }

      x->data[i] = b_a;
    } else {
      b_a = 0.0;
      loop_ub = a->size[1];
      for (i14 = 0; i14 < loop_ub; i14++) {
        b_a += a->data[i14] * f->data[i14];
      }

      x->data[i] = b_a;
    }
  }

  emxFree_real_T(&a);
}

void qpkwik(const emxArray_real_T *Linv, const emxArray_real_T *Hinv, const
            emxArray_real_T *f, const emxArray_real_T *Ac, const emxArray_real_T
            *b, emxArray_int16_T *iA, short m, short n, emxArray_real_T *x,
            emxArray_real_T *lambda, double *status)
{
  int b_status;
  int i11;
  int idx;
  emxArray_real_T *r;
  emxArray_real_T *RLinv;
  emxArray_real_T *D;
  emxArray_real_T *H;
  emxArray_real_T *cTol;
  double rMin;
  boolean_T cTolComputed;
  short iC_data[32767];
  short nA;
  double Xnorm0;
  emxArray_real_T *AcRow;
  emxArray_real_T *z;
  emxArray_real_T *y;
  emxArray_real_T *b_x;
  emxArray_real_T *a;
  emxArray_real_T *varargin_1;
  emxArray_real_T *b_a;
  int exitg2;
  double cMin;
  short kNext;
  int i;
  int inner;
  int exitg1;
  double cVal;
  int nx;
  unsigned int varargin_1_idx_0;
  boolean_T guard1 = false;
  int k;
  boolean_T exitg3;
  short kDrop;
  double t1;
  boolean_T isT1Inf;
  boolean_T tempOK;
  boolean_T b_guard1 = false;
  short i12;
  b_status = 1;
  i11 = lambda->size[0];
  lambda->size[0] = m;
  emxEnsureCapacity_real_T(lambda, i11);
  idx = m;
  for (i11 = 0; i11 < idx; i11++) {
    lambda->data[i11] = 0.0;
  }

  i11 = x->size[0];
  x->size[0] = n;
  emxEnsureCapacity_real_T(x, i11);
  idx = n;
  for (i11 = 0; i11 < idx; i11++) {
    x->data[i11] = 0.0;
  }

  if (m == 0) {
    Unconstrained(Hinv, f, x, n);
  } else {
    emxInit_real_T(&r, 1);
    i11 = r->size[0];
    r->size[0] = n;
    emxEnsureCapacity_real_T(r, i11);
    idx = n;
    for (i11 = 0; i11 < idx; i11++) {
      r->data[i11] = 0.0;
    }

    emxInit_real_T(&RLinv, 2);
    emxInit_real_T(&D, 2);
    emxInit_real_T(&H, 2);
    emxInit_real_T(&cTol, 1);
    rMin = 0.0;
    i11 = RLinv->size[0] * RLinv->size[1];
    RLinv->size[0] = Linv->size[0];
    RLinv->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(RLinv, i11);
    i11 = D->size[0] * D->size[1];
    D->size[0] = Linv->size[0];
    D->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(D, i11);
    i11 = H->size[0] * H->size[1];
    H->size[0] = Linv->size[0];
    H->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(H, i11);
    i11 = cTol->size[0];
    cTol->size[0] = m;
    emxEnsureCapacity_real_T(cTol, i11);
    idx = m;
    for (i11 = 0; i11 < idx; i11++) {
      cTol->data[i11] = 1.0;
    }

    cTolComputed = false;
    if (0 <= m - 1) {
      memset(&iC_data[0], 0, (unsigned int)(m * (int)sizeof(short)));
    }

    nA = 0;
    Unconstrained(Hinv, f, x, n);
    Xnorm0 = b_norm(x);
    emxInit_real_T(&AcRow, 2);
    emxInit_real_T(&z, 1);
    emxInit_real_T(&y, 2);
    emxInit_real_T(&b_x, 2);
    emxInit_real_T(&a, 2);
    emxInit_real_T(&varargin_1, 1);
    emxInit_real_T(&b_a, 2);
    do {
      exitg2 = 0;
      if (b_status <= 200) {
        cMin = -1.0E-6;
        kNext = 0;
        i11 = m;
        for (i = 0; i < i11; i++) {
          if (!cTolComputed) {
            idx = Ac->size[1];
            inner = b_x->size[0] * b_x->size[1];
            b_x->size[0] = 1;
            b_x->size[1] = idx;
            emxEnsureCapacity_real_T(b_x, inner);
            for (inner = 0; inner < idx; inner++) {
              b_x->data[inner] = Ac->data[i + Ac->size[0] * inner] * x->
                data[inner];
            }

            nx = b_x->size[1];
            inner = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = (unsigned short)b_x->size[1];
            emxEnsureCapacity_real_T(y, inner);
            for (k = 0; k < nx; k++) {
              y->data[k] = std::abs(b_x->data[k]);
            }

            nx = y->size[1];
            if (y->size[1] <= 2) {
              if (y->size[1] == 1) {
                cVal = y->data[0];
              } else if ((y->data[0] < y->data[1]) || (rtIsNaN(y->data[0]) &&
                          (!rtIsNaN(y->data[1])))) {
                cVal = y->data[1];
              } else {
                cVal = y->data[0];
              }
            } else {
              if (!rtIsNaN(y->data[0])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg3 = false;
                while ((!exitg3) && (k <= y->size[1])) {
                  if (!rtIsNaN(y->data[k - 1])) {
                    idx = k;
                    exitg3 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                cVal = y->data[0];
              } else {
                cVal = y->data[idx - 1];
                inner = idx + 1;
                for (k = inner; k <= nx; k++) {
                  if (cVal < y->data[k - 1]) {
                    cVal = y->data[k - 1];
                  }
                }
              }
            }

            if ((!(cTol->data[i] > cVal)) && (!rtIsNaN(cVal))) {
              cTol->data[i] = cVal;
            }
          }

          if (iA->data[i] == 0) {
            idx = Ac->size[1];
            inner = a->size[0] * a->size[1];
            a->size[0] = 1;
            a->size[1] = idx;
            emxEnsureCapacity_real_T(a, inner);
            for (inner = 0; inner < idx; inner++) {
              a->data[inner] = Ac->data[i + Ac->size[0] * inner];
            }

            inner = Ac->size[1];
            if ((inner == 1) || (x->size[0] == 1)) {
              cVal = 0.0;
              idx = a->size[1];
              for (inner = 0; inner < idx; inner++) {
                cVal += a->data[inner] * x->data[inner];
              }
            } else {
              cVal = 0.0;
              idx = a->size[1];
              for (inner = 0; inner < idx; inner++) {
                cVal += a->data[inner] * x->data[inner];
              }
            }

            cVal = (cVal - b->data[i]) / cTol->data[i];
            if (cVal < cMin) {
              cMin = cVal;
              kNext = (short)(1 + i);
            }
          }
        }

        cTolComputed = true;
        if (kNext <= 0) {
          exitg2 = 1;
        } else {
          do {
            exitg1 = 0;
            if ((kNext > 0) && (b_status <= 200)) {
              idx = Ac->size[1];
              i11 = AcRow->size[0] * AcRow->size[1];
              AcRow->size[0] = 1;
              AcRow->size[1] = idx;
              emxEnsureCapacity_real_T(AcRow, i11);
              for (i11 = 0; i11 < idx; i11++) {
                AcRow->data[i11] = Ac->data[(kNext + Ac->size[0] * i11) - 1];
              }

              guard1 = false;
              if (nA == 0) {
                i11 = varargin_1->size[0];
                varargin_1->size[0] = AcRow->size[1];
                emxEnsureCapacity_real_T(varargin_1, i11);
                idx = AcRow->size[1];
                for (i11 = 0; i11 < idx; i11++) {
                  varargin_1->data[i11] = AcRow->data[i11];
                }

                if ((Hinv->size[1] == 1) || (varargin_1->size[0] == 1)) {
                  i11 = z->size[0];
                  z->size[0] = Hinv->size[0];
                  emxEnsureCapacity_real_T(z, i11);
                  idx = Hinv->size[0];
                  for (i11 = 0; i11 < idx; i11++) {
                    z->data[i11] = 0.0;
                    nx = Hinv->size[1];
                    for (inner = 0; inner < nx; inner++) {
                      z->data[i11] += Hinv->data[i11 + Hinv->size[0] * inner] *
                        varargin_1->data[inner];
                    }
                  }
                } else {
                  idx = Hinv->size[0];
                  inner = Hinv->size[1];
                  i11 = z->size[0];
                  z->size[0] = Hinv->size[0];
                  emxEnsureCapacity_real_T(z, i11);
                  for (i = 0; i < idx; i++) {
                    z->data[i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    nx = k * idx;
                    for (i = 0; i < idx; i++) {
                      z->data[i] += varargin_1->data[k] * Hinv->data[nx + i];
                    }
                  }
                }

                guard1 = true;
              } else {
                cVal = KWIKfactor(Ac, iC_data, nA, Linv, RLinv, D, H, n);
                if (cVal <= 0.0) {
                  b_status = -2;
                  exitg1 = 1;
                } else {
                  i11 = b_a->size[0] * b_a->size[1];
                  b_a->size[0] = H->size[0];
                  b_a->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(b_a, i11);
                  idx = H->size[0] * H->size[1];
                  for (i11 = 0; i11 < idx; i11++) {
                    b_a->data[i11] = -H->data[i11];
                  }

                  i11 = varargin_1->size[0];
                  varargin_1->size[0] = AcRow->size[1];
                  emxEnsureCapacity_real_T(varargin_1, i11);
                  idx = AcRow->size[1];
                  for (i11 = 0; i11 < idx; i11++) {
                    varargin_1->data[i11] = AcRow->data[i11];
                  }

                  if ((b_a->size[1] == 1) || (varargin_1->size[0] == 1)) {
                    i11 = z->size[0];
                    z->size[0] = b_a->size[0];
                    emxEnsureCapacity_real_T(z, i11);
                    idx = b_a->size[0];
                    for (i11 = 0; i11 < idx; i11++) {
                      z->data[i11] = 0.0;
                      nx = b_a->size[1];
                      for (inner = 0; inner < nx; inner++) {
                        z->data[i11] += b_a->data[i11 + b_a->size[0] * inner] *
                          varargin_1->data[inner];
                      }
                    }
                  } else {
                    idx = b_a->size[0];
                    inner = b_a->size[1];
                    i11 = z->size[0];
                    z->size[0] = b_a->size[0];
                    emxEnsureCapacity_real_T(z, i11);
                    for (i = 0; i < idx; i++) {
                      z->data[i] = 0.0;
                    }

                    for (k = 0; k < inner; k++) {
                      nx = k * idx;
                      for (i = 0; i < idx; i++) {
                        z->data[i] += varargin_1->data[k] * b_a->data[nx + i];
                      }
                    }
                  }

                  i11 = nA;
                  for (i = 0; i < i11; i++) {
                    idx = D->size[0];
                    inner = varargin_1->size[0];
                    varargin_1->size[0] = idx;
                    emxEnsureCapacity_real_T(varargin_1, inner);
                    for (inner = 0; inner < idx; inner++) {
                      varargin_1->data[inner] = D->data[inner + D->size[0] * i];
                    }

                    inner = Ac->size[1];
                    b_guard1 = false;
                    if (inner == 1) {
                      b_guard1 = true;
                    } else {
                      inner = D->size[0];
                      if (inner == 1) {
                        b_guard1 = true;
                      } else {
                        cVal = 0.0;
                        idx = AcRow->size[1];
                        for (inner = 0; inner < idx; inner++) {
                          cVal += AcRow->data[inner] * varargin_1->data[inner];
                        }

                        r->data[i] = cVal;
                      }
                    }

                    if (b_guard1) {
                      cVal = 0.0;
                      idx = AcRow->size[1];
                      for (inner = 0; inner < idx; inner++) {
                        cVal += AcRow->data[inner] * varargin_1->data[inner];
                      }

                      r->data[i] = cVal;
                    }
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
                  nx = 0;
                  exitg3 = false;
                  while ((!exitg3) && (nx <= nA - 1)) {
                    if (r->data[nx] >= 1.0E-12) {
                      tempOK = false;
                      exitg3 = true;
                    } else {
                      nx++;
                    }
                  }
                }

                if ((nA == 0) || tempOK) {
                } else {
                  i11 = nA;
                  for (i = 0; i < i11; i++) {
                    if (r->data[i] > 1.0E-12) {
                      cVal = lambda->data[iC_data[i] - 1] / r->data[i];
                      if ((kDrop == 0) || (cVal < rMin)) {
                        rMin = cVal;
                        kDrop = (short)(1 + i);
                      }
                    }
                  }

                  if (kDrop > 0) {
                    t1 = rMin;
                    isT1Inf = false;
                  }
                }

                i11 = a->size[0] * a->size[1];
                a->size[0] = 1;
                a->size[1] = z->size[0];
                emxEnsureCapacity_real_T(a, i11);
                idx = z->size[0];
                for (i11 = 0; i11 < idx; i11++) {
                  a->data[i11] = z->data[i11];
                }

                i11 = varargin_1->size[0];
                varargin_1->size[0] = AcRow->size[1];
                emxEnsureCapacity_real_T(varargin_1, i11);
                idx = AcRow->size[1];
                for (i11 = 0; i11 < idx; i11++) {
                  varargin_1->data[i11] = AcRow->data[i11];
                }

                if ((a->size[1] == 1) || (varargin_1->size[0] == 1)) {
                  cMin = 0.0;
                  idx = a->size[1];
                  for (i11 = 0; i11 < idx; i11++) {
                    cMin += a->data[i11] * varargin_1->data[i11];
                  }
                } else {
                  cMin = 0.0;
                  idx = a->size[1];
                  for (i11 = 0; i11 < idx; i11++) {
                    cMin += a->data[i11] * varargin_1->data[i11];
                  }
                }

                if (cMin <= 0.0) {
                  cMin = 0.0;
                  tempOK = true;
                } else {
                  i11 = Ac->size[1];
                  if ((i11 == 1) || (x->size[0] == 1)) {
                    cVal = 0.0;
                    idx = AcRow->size[1];
                    for (i11 = 0; i11 < idx; i11++) {
                      cVal += AcRow->data[i11] * x->data[i11];
                    }
                  } else {
                    cVal = 0.0;
                    idx = AcRow->size[1];
                    for (i11 = 0; i11 < idx; i11++) {
                      cVal += AcRow->data[i11] * x->data[i11];
                    }
                  }

                  cMin = (b->data[kNext - 1] - cVal) / cMin;
                  tempOK = false;
                }

                if (isT1Inf && tempOK) {
                  b_status = -1;
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

                  i11 = nA;
                  for (i = 0; i < i11; i++) {
                    inner = iC_data[i] - 1;
                    lambda->data[inner] -= cVal * r->data[i];
                    if ((iC_data[i] <= m) && (lambda->data[inner] < 0.0)) {
                      lambda->data[inner] = 0.0;
                    }
                  }

                  lambda->data[kNext - 1] += cVal;
                  if (cVal == t1) {
                    DropConstraint(kDrop, iA, &nA, iC_data);
                  }

                  if (!tempOK) {
                    i11 = x->size[0];
                    emxEnsureCapacity_real_T(x, i11);
                    idx = x->size[0];
                    for (i11 = 0; i11 < idx; i11++) {
                      x->data[i11] += cVal * z->data[i11];
                    }

                    if (cVal == cMin) {
                      if (nA == n) {
                        b_status = -1;
                        exitg1 = 1;
                      } else {
                        i11 = nA + 1;
                        if (i11 > 32767) {
                          i11 = 32767;
                        }

                        nA = (short)i11;
                        iC_data[(short)i11 - 1] = kNext;
                        kDrop = (short)i11;
                        exitg3 = false;
                        while ((!exitg3) && (kDrop > 1)) {
                          i11 = kDrop - 1;
                          i12 = iC_data[i11];
                          inner = kDrop - 2;
                          if (iC_data[i11] > iC_data[inner]) {
                            exitg3 = true;
                          } else {
                            iC_data[i11] = iC_data[inner];
                            iC_data[inner] = i12;
                            kDrop = (short)i11;
                          }
                        }

                        iA->data[kNext - 1] = 1;
                        kNext = 0;
                        b_status++;
                      }
                    } else {
                      b_status++;
                    }
                  } else {
                    b_status++;
                  }
                }
              }
            } else {
              cVal = b_norm(x);
              if (std::abs(cVal - Xnorm0) > 0.001) {
                Xnorm0 = cVal;
                b_abs(b, varargin_1);
                varargin_1_idx_0 = (unsigned int)varargin_1->size[0];
                i11 = cTol->size[0];
                cTol->size[0] = (int)varargin_1_idx_0;
                emxEnsureCapacity_real_T(cTol, i11);
                varargin_1_idx_0 = (unsigned int)varargin_1->size[0];
                nx = (int)varargin_1_idx_0;
                for (k = 0; k < nx; k++) {
                  cVal = varargin_1->data[k];
                  if (!(cVal > 1.0)) {
                    cVal = 1.0;
                  }

                  cTol->data[k] = cVal;
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
        b_status = 0;
        exitg2 = 1;
      }
    } while (exitg2 == 0);

    emxFree_real_T(&b_a);
    emxFree_real_T(&varargin_1);
    emxFree_real_T(&a);
    emxFree_real_T(&b_x);
    emxFree_real_T(&y);
    emxFree_real_T(&z);
    emxFree_real_T(&AcRow);
    emxFree_real_T(&cTol);
    emxFree_real_T(&H);
    emxFree_real_T(&D);
    emxFree_real_T(&RLinv);
    emxFree_real_T(&r);
  }

  *status = b_status;
}

/* End of code generation (qpkwik.cpp) */
