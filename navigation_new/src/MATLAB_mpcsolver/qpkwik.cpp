//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpkwik.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "qpkwik.h"
#include "solveQP_emxutil.h"
#include "qr.h"
#include "abs.h"
#include "norm.h"

// Function Declarations
static void DropConstraint(short kDrop, emxArray_int16_T *iA, short *nA, short
  iC_data[]);
static double KWIKfactor(const emxArray_real_T *Ac, const short iC_data[], short
  nA, const emxArray_real_T *Linv, emxArray_real_T *RLinv, emxArray_real_T *D,
  emxArray_real_T *H, short n);
static void Unconstrained(const emxArray_real_T *Hinv, const emxArray_real_T *f,
  emxArray_real_T *x, short n);

// Function Definitions

//
// Arguments    : short kDrop
//                emxArray_int16_T *iA
//                short *nA
//                short iC_data[]
// Return Type  : void
//
static void DropConstraint(short kDrop, emxArray_int16_T *iA, short *nA, short
  iC_data[])
{
  int i15;
  short i16;
  short i;
  iA->data[iC_data[kDrop - 1] - 1] = 0;
  if (kDrop < *nA) {
    i15 = *nA - 1;
    if (i15 < -32768) {
      i15 = -32768;
    }

    i16 = (short)i15;
    for (i = kDrop; i <= i16; i++) {
      iC_data[i - 1] = iC_data[i];
    }
  }

  iC_data[*nA - 1] = 0;
  i15 = *nA - 1;
  if (i15 < -32768) {
    i15 = -32768;
  }

  *nA = (short)i15;
}

//
// Arguments    : const emxArray_real_T *Ac
//                const short iC_data[]
//                short nA
//                const emxArray_real_T *Linv
//                emxArray_real_T *RLinv
//                emxArray_real_T *D
//                emxArray_real_T *H
//                short n
// Return Type  : double
//
static double KWIKfactor(const emxArray_real_T *Ac, const short iC_data[], short
  nA, const emxArray_real_T *Linv, emxArray_real_T *RLinv, emxArray_real_T *D,
  emxArray_real_T *H, short n)
{
  double Status;
  emxArray_real_T *TL;
  int i11;
  int ar;
  int loop_ub;
  short i;
  int ia;
  emxArray_real_T *b;
  emxArray_real_T *C;
  emxArray_real_T *QQ;
  emxArray_real_T *RR;
  emxArray_real_T *a;
  int exitg1;
  int k;
  unsigned int Linv_idx_0;
  short j;
  int m;
  short b_k;
  int br;
  int ic;
  boolean_T guard1 = false;
  double y;
  emxInit_real_T(&TL, 2);
  i11 = TL->size[0] * TL->size[1];
  TL->size[0] = Linv->size[0];
  TL->size[1] = Linv->size[1];
  emxEnsureCapacity_real_T(TL, i11);
  Status = 1.0;
  ar = RLinv->size[0];
  loop_ub = RLinv->size[1];
  for (i11 = 0; i11 < loop_ub; i11++) {
    for (ia = 0; ia < ar; ia++) {
      RLinv->data[ia + RLinv->size[0] * i11] = 0.0;
    }
  }

  i = 1;
  emxInit_real_T1(&b, 1);
  emxInit_real_T1(&C, 1);
  while (i <= nA) {
    ar = Ac->size[1];
    i11 = b->size[0];
    b->size[0] = ar;
    emxEnsureCapacity_real_T1(b, i11);
    for (i11 = 0; i11 < ar; i11++) {
      b->data[i11] = Ac->data[(iC_data[i - 1] + Ac->size[0] * i11) - 1];
    }

    if ((Linv->size[1] == 1) || (b->size[0] == 1)) {
      i11 = C->size[0];
      C->size[0] = Linv->size[0];
      emxEnsureCapacity_real_T1(C, i11);
      ar = Linv->size[0];
      for (i11 = 0; i11 < ar; i11++) {
        C->data[i11] = 0.0;
        loop_ub = Linv->size[1];
        for (ia = 0; ia < loop_ub; ia++) {
          C->data[i11] += Linv->data[i11 + Linv->size[0] * ia] * b->data[ia];
        }
      }
    } else {
      k = Linv->size[1];
      Linv_idx_0 = (unsigned int)Linv->size[0];
      i11 = C->size[0];
      C->size[0] = (int)Linv_idx_0;
      emxEnsureCapacity_real_T1(C, i11);
      m = Linv->size[0];
      ar = C->size[0];
      i11 = C->size[0];
      C->size[0] = ar;
      emxEnsureCapacity_real_T1(C, i11);
      for (i11 = 0; i11 < ar; i11++) {
        C->data[i11] = 0.0;
      }

      if (Linv->size[0] != 0) {
        ar = 0;
        while ((m > 0) && (ar <= 0)) {
          for (ic = 1; ic <= m; ic++) {
            C->data[ic - 1] = 0.0;
          }

          ar = m;
        }

        br = 0;
        ar = 0;
        while ((m > 0) && (ar <= 0)) {
          ar = -1;
          i11 = br + k;
          for (loop_ub = br; loop_ub + 1 <= i11; loop_ub++) {
            if (b->data[loop_ub] != 0.0) {
              ia = ar;
              for (ic = 0; ic + 1 <= m; ic++) {
                ia++;
                C->data[ic] += b->data[loop_ub] * Linv->data[ia];
              }
            }

            ar += m;
          }

          br += k;
          ar = m;
        }
      }
    }

    ar = C->size[0];
    for (i11 = 0; i11 < ar; i11++) {
      RLinv->data[i11 + RLinv->size[0] * (i - 1)] = C->data[i11];
    }

    i++;
  }

  emxFree_real_T(&C);
  emxInit_real_T(&QQ, 2);
  emxInit_real_T(&RR, 2);
  qr(RLinv, QQ, RR);
  i = 1;
  emxInit_real_T(&a, 2);
  do {
    exitg1 = 0;
    if (i <= nA) {
      if (std::abs(RR->data[(i + RR->size[0] * (i - 1)) - 1]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (i = 1; i <= n; i++) {
        for (j = 1; j <= n; j++) {
          ar = Linv->size[0];
          i11 = a->size[0] * a->size[1];
          a->size[0] = 1;
          a->size[1] = ar;
          emxEnsureCapacity_real_T(a, i11);
          for (i11 = 0; i11 < ar; i11++) {
            a->data[a->size[0] * i11] = Linv->data[i11 + Linv->size[0] * (i - 1)];
          }

          ar = QQ->size[0];
          i11 = b->size[0];
          b->size[0] = ar;
          emxEnsureCapacity_real_T1(b, i11);
          for (i11 = 0; i11 < ar; i11++) {
            b->data[i11] = QQ->data[i11 + QQ->size[0] * (j - 1)];
          }

          guard1 = false;
          if (a->size[1] == 1) {
            guard1 = true;
          } else {
            i11 = QQ->size[0];
            if (i11 == 1) {
              guard1 = true;
            } else {
              y = 0.0;
              for (i11 = 0; i11 < a->size[1]; i11++) {
                y += a->data[a->size[0] * i11] * b->data[i11];
              }
            }
          }

          if (guard1) {
            y = 0.0;
            for (i11 = 0; i11 < a->size[1]; i11++) {
              y += a->data[a->size[0] * i11] * b->data[i11];
            }
          }

          TL->data[(i + TL->size[0] * (j - 1)) - 1] = y;
        }
      }

      ar = RLinv->size[0];
      loop_ub = RLinv->size[1];
      for (i11 = 0; i11 < loop_ub; i11++) {
        for (ia = 0; ia < ar; ia++) {
          RLinv->data[ia + RLinv->size[0] * i11] = 0.0;
        }
      }

      for (j = nA; j > 0; j--) {
        RLinv->data[(j + RLinv->size[0] * (j - 1)) - 1] = 1.0;
        for (b_k = j; b_k <= nA; b_k++) {
          RLinv->data[(j + RLinv->size[0] * (b_k - 1)) - 1] /= RR->data[(j +
            RR->size[0] * (j - 1)) - 1];
        }

        if (j > 1) {
          for (i = 1; i < j; i++) {
            for (b_k = j; b_k <= nA; b_k++) {
              RLinv->data[(i + RLinv->size[0] * (b_k - 1)) - 1] -= RR->data[(i +
                RR->size[0] * (j - 1)) - 1] * RLinv->data[(j + RLinv->size[0] *
                (b_k - 1)) - 1];
            }
          }
        }
      }

      for (i = 1; i <= n; i++) {
        for (j = i; j <= n; j++) {
          H->data[(i + H->size[0] * (j - 1)) - 1] = 0.0;
          for (b_k = (short)(nA + 1); b_k <= n; b_k++) {
            H->data[(i + H->size[0] * (j - 1)) - 1] -= TL->data[(i + TL->size[0]
              * (b_k - 1)) - 1] * TL->data[(j + TL->size[0] * (b_k - 1)) - 1];
          }

          H->data[(j + H->size[0] * (i - 1)) - 1] = H->data[(i + H->size[0] * (j
            - 1)) - 1];
        }
      }

      for (j = 1; j <= nA; j++) {
        for (i = 1; i <= n; i++) {
          D->data[(i + D->size[0] * (j - 1)) - 1] = 0.0;
          for (b_k = j; b_k <= nA; b_k++) {
            D->data[(i + D->size[0] * (j - 1)) - 1] += TL->data[(i + TL->size[0]
              * (b_k - 1)) - 1] * RLinv->data[(j + RLinv->size[0] * (b_k - 1)) -
              1];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&b);
  emxFree_real_T(&a);
  emxFree_real_T(&RR);
  emxFree_real_T(&QQ);
  emxFree_real_T(&TL);
  return Status;
}

//
// Arguments    : const emxArray_real_T *Hinv
//                const emxArray_real_T *f
//                emxArray_real_T *x
//                short n
// Return Type  : void
//
static void Unconstrained(const emxArray_real_T *Hinv, const emxArray_real_T *f,
  emxArray_real_T *x, short n)
{
  short i;
  emxArray_real_T *a;
  int loop_ub;
  int i10;
  double y;
  i = 1;
  emxInit_real_T(&a, 2);
  while (i <= n) {
    loop_ub = Hinv->size[1];
    i10 = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = loop_ub;
    emxEnsureCapacity_real_T(a, i10);
    for (i10 = 0; i10 < loop_ub; i10++) {
      a->data[a->size[0] * i10] = -Hinv->data[(i + Hinv->size[0] * i10) - 1];
    }

    if ((a->size[1] == 1) || (f->size[0] == 1)) {
      y = 0.0;
      for (i10 = 0; i10 < a->size[1]; i10++) {
        y += a->data[a->size[0] * i10] * f->data[i10];
      }
    } else {
      y = 0.0;
      for (i10 = 0; i10 < a->size[1]; i10++) {
        y += a->data[a->size[0] * i10] * f->data[i10];
      }
    }

    x->data[i - 1] = y;
    i++;
  }

  emxFree_real_T(&a);
}

//
// Arguments    : const emxArray_real_T *Linv
//                const emxArray_real_T *Hinv
//                const emxArray_real_T *f
//                const emxArray_real_T *Ac
//                const emxArray_real_T *b
//                emxArray_int16_T *iA
//                short m
//                short n
//                emxArray_real_T *x
//                emxArray_real_T *lambda
//                double *status
// Return Type  : void
//
void qpkwik(const emxArray_real_T *Linv, const emxArray_real_T *Hinv, const
            emxArray_real_T *f, const emxArray_real_T *Ac, const emxArray_real_T
            *b, emxArray_int16_T *iA, short m, short n, emxArray_real_T *x,
            emxArray_real_T *lambda, double *status)
{
  int b_status;
  int i9;
  int ar;
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
  emxArray_real_T *varargin_1;
  emxArray_real_T *a;
  emxArray_real_T *b_a;
  emxArray_real_T *y;
  emxArray_real_T *b_x;
  int exitg2;
  double cMin;
  short kNext;
  short i;
  int exitg1;
  double Xnorm;
  unsigned int varargin_1_idx_0;
  unsigned int b_varargin_1_idx_0;
  boolean_T guard1 = false;
  double b_y;
  int k;
  int b_n;
  int ia;
  short kDrop;
  boolean_T exitg3;
  double t1;
  boolean_T isT1Inf;
  boolean_T tempOK;
  short iSave;
  int b_m;
  int br;
  int ic;
  boolean_T b_guard1 = false;
  b_status = 1;
  i9 = lambda->size[0];
  lambda->size[0] = m;
  emxEnsureCapacity_real_T1(lambda, i9);
  ar = m;
  for (i9 = 0; i9 < ar; i9++) {
    lambda->data[i9] = 0.0;
  }

  i9 = x->size[0];
  x->size[0] = n;
  emxEnsureCapacity_real_T1(x, i9);
  ar = n;
  for (i9 = 0; i9 < ar; i9++) {
    x->data[i9] = 0.0;
  }

  if (m == 0) {
    Unconstrained(Hinv, f, x, n);
  } else {
    emxInit_real_T1(&r, 1);
    i9 = r->size[0];
    r->size[0] = n;
    emxEnsureCapacity_real_T1(r, i9);
    ar = n;
    for (i9 = 0; i9 < ar; i9++) {
      r->data[i9] = 0.0;
    }

    emxInit_real_T(&RLinv, 2);
    emxInit_real_T(&D, 2);
    emxInit_real_T(&H, 2);
    emxInit_real_T1(&cTol, 1);
    rMin = 0.0;
    i9 = RLinv->size[0] * RLinv->size[1];
    RLinv->size[0] = Linv->size[0];
    RLinv->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(RLinv, i9);
    i9 = D->size[0] * D->size[1];
    D->size[0] = Linv->size[0];
    D->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(D, i9);
    i9 = H->size[0] * H->size[1];
    H->size[0] = Linv->size[0];
    H->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(H, i9);
    i9 = cTol->size[0];
    cTol->size[0] = m;
    emxEnsureCapacity_real_T1(cTol, i9);
    ar = m;
    for (i9 = 0; i9 < ar; i9++) {
      cTol->data[i9] = 1.0;
    }

    cTolComputed = false;
    ar = m;
    if (0 <= ar - 1) {
      memset(&iC_data[0], 0, (unsigned int)(ar * (int)sizeof(short)));
    }

    nA = 0;
    Unconstrained(Hinv, f, x, n);
    Xnorm0 = norm(x);
    emxInit_real_T(&AcRow, 2);
    emxInit_real_T1(&z, 1);
    emxInit_real_T1(&varargin_1, 1);
    emxInit_real_T(&a, 2);
    emxInit_real_T(&b_a, 2);
    emxInit_real_T(&y, 2);
    emxInit_real_T(&b_x, 2);
    do {
      exitg2 = 0;
      if (b_status <= 200) {
        cMin = -1.0E-6;
        kNext = 0;
        for (i = 1; i <= m; i++) {
          if (!cTolComputed) {
            ar = Ac->size[1];
            i9 = b_x->size[0] * b_x->size[1];
            b_x->size[0] = 1;
            b_x->size[1] = ar;
            emxEnsureCapacity_real_T(b_x, i9);
            for (i9 = 0; i9 < ar; i9++) {
              b_x->data[b_x->size[0] * i9] = Ac->data[(i + Ac->size[0] * i9) - 1]
                * x->data[i9];
            }

            i9 = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = (unsigned short)b_x->size[1];
            emxEnsureCapacity_real_T(y, i9);
            for (k = 0; k + 1 <= b_x->size[1]; k++) {
              y->data[k] = std::abs(b_x->data[k]);
            }

            ar = 1;
            b_n = y->size[1];
            Xnorm = y->data[0];
            if (y->size[1] > 1) {
              if (rtIsNaN(y->data[0])) {
                ia = 2;
                exitg3 = false;
                while ((!exitg3) && (ia <= b_n)) {
                  ar = ia;
                  if (!rtIsNaN(y->data[ia - 1])) {
                    Xnorm = y->data[ia - 1];
                    exitg3 = true;
                  } else {
                    ia++;
                  }
                }
              }

              if (ar < y->size[1]) {
                while (ar + 1 <= b_n) {
                  if (y->data[ar] > Xnorm) {
                    Xnorm = y->data[ar];
                  }

                  ar++;
                }
              }
            }

            if ((cTol->data[i - 1] > Xnorm) || rtIsNaN(Xnorm)) {
              Xnorm = cTol->data[i - 1];
            }

            cTol->data[i - 1] = Xnorm;
          }

          if (iA->data[i - 1] == 0) {
            ar = Ac->size[1];
            i9 = b_a->size[0] * b_a->size[1];
            b_a->size[0] = 1;
            b_a->size[1] = ar;
            emxEnsureCapacity_real_T(b_a, i9);
            for (i9 = 0; i9 < ar; i9++) {
              b_a->data[b_a->size[0] * i9] = Ac->data[(i + Ac->size[0] * i9) - 1];
            }

            i9 = Ac->size[1];
            if ((i9 == 1) || (x->size[0] == 1)) {
              b_y = 0.0;
              for (i9 = 0; i9 < b_a->size[1]; i9++) {
                b_y += b_a->data[b_a->size[0] * i9] * x->data[i9];
              }
            } else {
              b_y = 0.0;
              for (i9 = 0; i9 < b_a->size[1]; i9++) {
                b_y += b_a->data[b_a->size[0] * i9] * x->data[i9];
              }
            }

            Xnorm = (b_y - b->data[i - 1]) / cTol->data[i - 1];
            if (Xnorm < cMin) {
              cMin = Xnorm;
              kNext = i;
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
              ar = Ac->size[1];
              i9 = AcRow->size[0] * AcRow->size[1];
              AcRow->size[0] = 1;
              AcRow->size[1] = ar;
              emxEnsureCapacity_real_T(AcRow, i9);
              for (i9 = 0; i9 < ar; i9++) {
                AcRow->data[AcRow->size[0] * i9] = Ac->data[(kNext + Ac->size[0]
                  * i9) - 1];
              }

              guard1 = false;
              if (nA == 0) {
                i9 = varargin_1->size[0];
                varargin_1->size[0] = AcRow->size[1];
                emxEnsureCapacity_real_T1(varargin_1, i9);
                ar = AcRow->size[1];
                for (i9 = 0; i9 < ar; i9++) {
                  varargin_1->data[i9] = AcRow->data[AcRow->size[0] * i9];
                }

                if ((Hinv->size[1] == 1) || (varargin_1->size[0] == 1)) {
                  i9 = z->size[0];
                  z->size[0] = Hinv->size[0];
                  emxEnsureCapacity_real_T1(z, i9);
                  ar = Hinv->size[0];
                  for (i9 = 0; i9 < ar; i9++) {
                    z->data[i9] = 0.0;
                    b_n = Hinv->size[1];
                    for (ia = 0; ia < b_n; ia++) {
                      z->data[i9] += Hinv->data[i9 + Hinv->size[0] * ia] *
                        varargin_1->data[ia];
                    }
                  }
                } else {
                  k = Hinv->size[1];
                  varargin_1_idx_0 = (unsigned int)Hinv->size[0];
                  i9 = z->size[0];
                  z->size[0] = (int)varargin_1_idx_0;
                  emxEnsureCapacity_real_T1(z, i9);
                  b_m = Hinv->size[0];
                  ar = z->size[0];
                  i9 = z->size[0];
                  z->size[0] = ar;
                  emxEnsureCapacity_real_T1(z, i9);
                  for (i9 = 0; i9 < ar; i9++) {
                    z->data[i9] = 0.0;
                  }

                  if (Hinv->size[0] != 0) {
                    ar = 0;
                    while ((b_m > 0) && (ar <= 0)) {
                      for (ic = 1; ic <= b_m; ic++) {
                        z->data[ic - 1] = 0.0;
                      }

                      ar = b_m;
                    }

                    br = 0;
                    ar = 0;
                    while ((b_m > 0) && (ar <= 0)) {
                      ar = -1;
                      i9 = br + k;
                      for (b_n = br; b_n + 1 <= i9; b_n++) {
                        if (varargin_1->data[b_n] != 0.0) {
                          ia = ar;
                          for (ic = 0; ic + 1 <= b_m; ic++) {
                            ia++;
                            z->data[ic] += varargin_1->data[b_n] * Hinv->data[ia];
                          }
                        }

                        ar += b_m;
                      }

                      br += k;
                      ar = b_m;
                    }
                  }
                }

                guard1 = true;
              } else {
                Xnorm = KWIKfactor(Ac, iC_data, nA, Linv, RLinv, D, H, n);
                if (Xnorm <= 0.0) {
                  b_status = -2;
                  exitg1 = 1;
                } else {
                  i9 = a->size[0] * a->size[1];
                  a->size[0] = H->size[0];
                  a->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(a, i9);
                  ar = H->size[0] * H->size[1];
                  for (i9 = 0; i9 < ar; i9++) {
                    a->data[i9] = -H->data[i9];
                  }

                  i9 = varargin_1->size[0];
                  varargin_1->size[0] = AcRow->size[1];
                  emxEnsureCapacity_real_T1(varargin_1, i9);
                  ar = AcRow->size[1];
                  for (i9 = 0; i9 < ar; i9++) {
                    varargin_1->data[i9] = AcRow->data[AcRow->size[0] * i9];
                  }

                  if ((a->size[1] == 1) || (varargin_1->size[0] == 1)) {
                    i9 = z->size[0];
                    z->size[0] = a->size[0];
                    emxEnsureCapacity_real_T1(z, i9);
                    ar = a->size[0];
                    for (i9 = 0; i9 < ar; i9++) {
                      z->data[i9] = 0.0;
                      b_n = a->size[1];
                      for (ia = 0; ia < b_n; ia++) {
                        z->data[i9] += a->data[i9 + a->size[0] * ia] *
                          varargin_1->data[ia];
                      }
                    }
                  } else {
                    k = a->size[1];
                    varargin_1_idx_0 = (unsigned int)a->size[0];
                    i9 = z->size[0];
                    z->size[0] = (int)varargin_1_idx_0;
                    emxEnsureCapacity_real_T1(z, i9);
                    b_m = a->size[0];
                    ar = z->size[0];
                    i9 = z->size[0];
                    z->size[0] = ar;
                    emxEnsureCapacity_real_T1(z, i9);
                    for (i9 = 0; i9 < ar; i9++) {
                      z->data[i9] = 0.0;
                    }

                    if (a->size[0] != 0) {
                      ar = 0;
                      while ((b_m > 0) && (ar <= 0)) {
                        for (ic = 1; ic <= b_m; ic++) {
                          z->data[ic - 1] = 0.0;
                        }

                        ar = b_m;
                      }

                      br = 0;
                      ar = 0;
                      while ((b_m > 0) && (ar <= 0)) {
                        ar = -1;
                        i9 = br + k;
                        for (b_n = br; b_n + 1 <= i9; b_n++) {
                          if (varargin_1->data[b_n] != 0.0) {
                            ia = ar;
                            for (ic = 0; ic + 1 <= b_m; ic++) {
                              ia++;
                              z->data[ic] += varargin_1->data[b_n] * a->data[ia];
                            }
                          }

                          ar += b_m;
                        }

                        br += k;
                        ar = b_m;
                      }
                    }
                  }

                  for (i = 1; i <= nA; i++) {
                    ar = D->size[0];
                    i9 = varargin_1->size[0];
                    varargin_1->size[0] = ar;
                    emxEnsureCapacity_real_T1(varargin_1, i9);
                    for (i9 = 0; i9 < ar; i9++) {
                      varargin_1->data[i9] = D->data[i9 + D->size[0] * (i - 1)];
                    }

                    i9 = Ac->size[1];
                    b_guard1 = false;
                    if (i9 == 1) {
                      b_guard1 = true;
                    } else {
                      i9 = D->size[0];
                      if (i9 == 1) {
                        b_guard1 = true;
                      } else {
                        b_y = 0.0;
                        for (i9 = 0; i9 < AcRow->size[1]; i9++) {
                          b_y += AcRow->data[AcRow->size[0] * i9] *
                            varargin_1->data[i9];
                        }
                      }
                    }

                    if (b_guard1) {
                      b_y = 0.0;
                      for (i9 = 0; i9 < AcRow->size[1]; i9++) {
                        b_y += AcRow->data[AcRow->size[0] * i9] *
                          varargin_1->data[i9];
                      }
                    }

                    r->data[i - 1] = b_y;
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
                    if (r->data[iSave - 1] >= 1.0E-12) {
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
                  for (i = 1; i <= nA; i++) {
                    if (r->data[i - 1] > 1.0E-12) {
                      Xnorm = lambda->data[iC_data[i - 1] - 1] / r->data[i - 1];
                      if ((kDrop == 0) || (Xnorm < rMin)) {
                        rMin = Xnorm;
                        kDrop = i;
                      }
                    }
                  }

                  if (kDrop > 0) {
                    t1 = rMin;
                    isT1Inf = false;
                  }
                }

                i9 = b_a->size[0] * b_a->size[1];
                b_a->size[0] = 1;
                b_a->size[1] = z->size[0];
                emxEnsureCapacity_real_T(b_a, i9);
                ar = z->size[0];
                for (i9 = 0; i9 < ar; i9++) {
                  b_a->data[b_a->size[0] * i9] = z->data[i9];
                }

                i9 = varargin_1->size[0];
                varargin_1->size[0] = AcRow->size[1];
                emxEnsureCapacity_real_T1(varargin_1, i9);
                ar = AcRow->size[1];
                for (i9 = 0; i9 < ar; i9++) {
                  varargin_1->data[i9] = AcRow->data[AcRow->size[0] * i9];
                }

                if ((b_a->size[1] == 1) || (varargin_1->size[0] == 1)) {
                  Xnorm = 0.0;
                  for (i9 = 0; i9 < b_a->size[1]; i9++) {
                    Xnorm += b_a->data[b_a->size[0] * i9] * varargin_1->data[i9];
                  }
                } else {
                  Xnorm = 0.0;
                  for (i9 = 0; i9 < b_a->size[1]; i9++) {
                    Xnorm += b_a->data[b_a->size[0] * i9] * varargin_1->data[i9];
                  }
                }

                if (Xnorm <= 0.0) {
                  cMin = 0.0;
                  tempOK = true;
                } else {
                  i9 = Ac->size[1];
                  if ((i9 == 1) || (x->size[0] == 1)) {
                    b_y = 0.0;
                    for (i9 = 0; i9 < AcRow->size[1]; i9++) {
                      b_y += AcRow->data[AcRow->size[0] * i9] * x->data[i9];
                    }
                  } else {
                    b_y = 0.0;
                    for (i9 = 0; i9 < AcRow->size[1]; i9++) {
                      b_y += AcRow->data[AcRow->size[0] * i9] * x->data[i9];
                    }
                  }

                  cMin = (b->data[kNext - 1] - b_y) / Xnorm;
                  tempOK = false;
                }

                if (isT1Inf && tempOK) {
                  b_status = -1;
                  exitg1 = 1;
                } else {
                  if ((t1 < cMin) || rtIsNaN(cMin)) {
                    b_y = t1;
                  } else {
                    b_y = cMin;
                  }

                  if (tempOK) {
                    Xnorm = t1;
                  } else if (isT1Inf) {
                    Xnorm = cMin;
                  } else {
                    Xnorm = b_y;
                  }

                  for (i = 1; i <= nA; i++) {
                    lambda->data[iC_data[i - 1] - 1] -= Xnorm * r->data[i - 1];
                    if ((iC_data[i - 1] <= m) && (lambda->data[iC_data[i - 1] -
                         1] < 0.0)) {
                      lambda->data[iC_data[i - 1] - 1] = 0.0;
                    }
                  }

                  lambda->data[kNext - 1] += Xnorm;
                  if (Xnorm == t1) {
                    DropConstraint(kDrop, iA, &nA, iC_data);
                  }

                  if (!tempOK) {
                    i9 = x->size[0];
                    emxEnsureCapacity_real_T1(x, i9);
                    ar = x->size[0];
                    for (i9 = 0; i9 < ar; i9++) {
                      x->data[i9] += Xnorm * z->data[i9];
                    }

                    if (Xnorm == cMin) {
                      if (nA == n) {
                        b_status = -1;
                        exitg1 = 1;
                      } else {
                        i9 = nA + 1;
                        if (i9 > 32767) {
                          i9 = 32767;
                        }

                        nA = (short)i9;
                        iC_data[nA - 1] = kNext;
                        i = nA;
                        while ((i > 1) && (!(iC_data[i - 1] > iC_data[i - 2])))
                        {
                          iSave = iC_data[i - 1];
                          iC_data[i - 1] = iC_data[i - 2];
                          iC_data[i - 2] = iSave;
                          i--;
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
              Xnorm = norm(x);
              if (std::abs(Xnorm - Xnorm0) > 0.001) {
                Xnorm0 = Xnorm;
                b_abs(b, varargin_1);
                varargin_1_idx_0 = (unsigned int)varargin_1->size[0];
                b_varargin_1_idx_0 = (unsigned int)varargin_1->size[0];
                i9 = cTol->size[0];
                cTol->size[0] = (int)b_varargin_1_idx_0;
                emxEnsureCapacity_real_T1(cTol, i9);
                for (k = 0; k + 1 <= (int)varargin_1_idx_0; k++) {
                  Xnorm = varargin_1->data[k];
                  if (!(Xnorm > 1.0)) {
                    Xnorm = 1.0;
                  }

                  cTol->data[k] = Xnorm;
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

    emxFree_real_T(&b_x);
    emxFree_real_T(&y);
    emxFree_real_T(&b_a);
    emxFree_real_T(&a);
    emxFree_real_T(&varargin_1);
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

//
// File trailer for qpkwik.cpp
//
// [EOF]
//
