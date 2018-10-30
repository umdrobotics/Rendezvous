//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpcqpsolver.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "mpcqpsolver.h"
#include "solveQP_emxutil.h"
#include "qpkwik.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *Linv
//                const emxArray_real_T *f
//                const emxArray_real_T *A
//                const emxArray_real_T *b
//                const emxArray_boolean_T *iA0
//                emxArray_real_T *x
//                double *status
// Return Type  : void
//
void mpcqpsolver(const emxArray_real_T *Linv, const emxArray_real_T *f, const
                 emxArray_real_T *A, const emxArray_real_T *b, const
                 emxArray_boolean_T *iA0, emxArray_real_T *x, double *status)
{
  emxArray_real_T *a;
  int i6;
  int ar;
  emxArray_real_T *Hinv;
  int br;
  int i7;
  int k;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int m;
  int c;
  int cr;
  cell_wrap_0 reshapes[2];
  int ic;
  emxArray_int16_T *iA1;
  int ib;
  int ia;
  emxArray_real_T *lam;
  double b_status;
  emxInit_real_T(&a, 2);
  i6 = a->size[0] * a->size[1];
  a->size[0] = Linv->size[1];
  a->size[1] = Linv->size[0];
  emxEnsureCapacity_real_T(a, i6);
  ar = Linv->size[0];
  for (i6 = 0; i6 < ar; i6++) {
    br = Linv->size[1];
    for (i7 = 0; i7 < br; i7++) {
      a->data[i7 + a->size[0] * i6] = Linv->data[i6 + Linv->size[0] * i7];
    }
  }

  emxInit_real_T(&Hinv, 2);
  if ((a->size[1] == 1) || (Linv->size[0] == 1)) {
    i6 = Hinv->size[0] * Hinv->size[1];
    Hinv->size[0] = a->size[0];
    Hinv->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(Hinv, i6);
    ar = a->size[0];
    for (i6 = 0; i6 < ar; i6++) {
      br = Linv->size[1];
      for (i7 = 0; i7 < br; i7++) {
        Hinv->data[i6 + Hinv->size[0] * i7] = 0.0;
        c = a->size[1];
        for (cr = 0; cr < c; cr++) {
          Hinv->data[i6 + Hinv->size[0] * i7] += a->data[i6 + a->size[0] * cr] *
            Linv->data[cr + Linv->size[0] * i7];
        }
      }
    }
  } else {
    k = a->size[1];
    unnamed_idx_0 = (unsigned int)a->size[0];
    unnamed_idx_1 = (unsigned int)Linv->size[1];
    i6 = Hinv->size[0] * Hinv->size[1];
    Hinv->size[0] = (int)unnamed_idx_0;
    Hinv->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(Hinv, i6);
    m = a->size[0];
    i6 = Hinv->size[0] * Hinv->size[1];
    emxEnsureCapacity_real_T(Hinv, i6);
    ar = Hinv->size[1];
    for (i6 = 0; i6 < ar; i6++) {
      br = Hinv->size[0];
      for (i7 = 0; i7 < br; i7++) {
        Hinv->data[i7 + Hinv->size[0] * i6] = 0.0;
      }
    }

    if ((a->size[0] == 0) || (Linv->size[1] == 0)) {
    } else {
      c = a->size[0] * (Linv->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i6 = cr + m;
        for (ic = cr; ic + 1 <= i6; ic++) {
          Hinv->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = 0;
        i6 = br + k;
        for (ib = br; ib + 1 <= i6; ib++) {
          if (Linv->data[ib] != 0.0) {
            ia = ar;
            i7 = cr + m;
            for (ic = cr; ic + 1 <= i7; ic++) {
              ia++;
              Hinv->data[ic] += Linv->data[ib] * a->data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  if (!((A->size[0] == 0) || (A->size[1] == 0))) {
    c = A->size[1];
  } else {
    c = 0;
  }

  if ((c == 0) || (!((A->size[0] == 0) || (A->size[1] == 0)))) {
    cr = A->size[0];
  } else {
    cr = 0;
  }

  emxInitMatrix_cell_wrap_0(reshapes);
  i6 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
  reshapes[1].f1->size[0] = 0;
  reshapes[1].f1->size[1] = c;
  emxEnsureCapacity_real_T(reshapes[1].f1, i6);
  emxInit_int16_T(&iA1, 1);
  i6 = iA1->size[0];
  iA1->size[0] = iA0->size[0];
  emxEnsureCapacity_int16_T(iA1, i6);
  ar = iA0->size[0];
  for (i6 = 0; i6 < ar; i6++) {
    iA1->data[i6] = iA0->data[i6];
  }

  i6 = a->size[0] * a->size[1];
  a->size[0] = cr + reshapes[1].f1->size[0];
  a->size[1] = c;
  emxEnsureCapacity_real_T(a, i6);
  for (i6 = 0; i6 < c; i6++) {
    for (i7 = 0; i7 < cr; i7++) {
      a->data[i7 + a->size[0] * i6] = A->data[i7 + cr * i6];
    }
  }

  ar = reshapes[1].f1->size[1];
  for (i6 = 0; i6 < ar; i6++) {
    br = reshapes[1].f1->size[0];
    for (i7 = 0; i7 < br; i7++) {
      a->data[(i7 + cr) + a->size[0] * i6] = reshapes[1].f1->data[i7 + reshapes
        [1].f1->size[0] * i6];
    }
  }

  emxFreeMatrix_cell_wrap_0(reshapes);
  emxInit_real_T1(&lam, 1);
  i6 = A->size[0];
  if (i6 > 32767) {
    i6 = 32767;
  } else {
    if (i6 < -32768) {
      i6 = -32768;
    }
  }

  i7 = Linv->size[0];
  if (i7 > 32767) {
    i7 = 32767;
  } else {
    if (i7 < -32768) {
      i7 = -32768;
    }
  }

  qpkwik(Linv, Hinv, f, a, b, iA1, (short)i6, (short)i7, x, lam, &b_status);
  *status = b_status;
  emxFree_real_T(&a);
  emxFree_int16_T(&iA1);
  emxFree_real_T(&lam);
  emxFree_real_T(&Hinv);
}

//
// File trailer for mpcqpsolver.cpp
//
// [EOF]
//
