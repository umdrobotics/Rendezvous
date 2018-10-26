/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mpcqpsolver.cpp
 *
 * Code generation for function 'mpcqpsolver'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "mpcqpsolver.h"
#include "solveQP_emxutil.h"
#include "qpkwik.h"

/* Function Definitions */
void mpcqpsolver(const emxArray_real_T *Linv, const emxArray_real_T *f, const
                 emxArray_real_T *A, const emxArray_real_T *b, const
                 emxArray_boolean_T *iA0, emxArray_real_T *x, double *status)
{
  emxArray_real_T *a;
  int i9;
  int loop_ub;
  emxArray_real_T *Hinv;
  int n;
  int coffset;
  int m;
  int inner;
  int boffset;
  int i;
  int k;
  cell_wrap_0 reshapes[2];
  int aoffset;
  double temp;
  emxArray_int16_T *iA1;
  emxArray_real_T *lam;
  emxInit_real_T(&a, 2);
  i9 = a->size[0] * a->size[1];
  a->size[0] = Linv->size[1];
  a->size[1] = Linv->size[0];
  emxEnsureCapacity_real_T(a, i9);
  loop_ub = Linv->size[0];
  for (i9 = 0; i9 < loop_ub; i9++) {
    n = Linv->size[1];
    for (coffset = 0; coffset < n; coffset++) {
      a->data[coffset + a->size[0] * i9] = Linv->data[i9 + Linv->size[0] *
        coffset];
    }
  }

  emxInit_real_T(&Hinv, 2);
  if ((a->size[1] == 1) || (Linv->size[0] == 1)) {
    i9 = Hinv->size[0] * Hinv->size[1];
    Hinv->size[0] = a->size[0];
    Hinv->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(Hinv, i9);
    loop_ub = a->size[0];
    for (i9 = 0; i9 < loop_ub; i9++) {
      n = Linv->size[1];
      for (coffset = 0; coffset < n; coffset++) {
        Hinv->data[i9 + Hinv->size[0] * coffset] = 0.0;
        m = a->size[1];
        for (inner = 0; inner < m; inner++) {
          Hinv->data[i9 + Hinv->size[0] * coffset] += a->data[i9 + a->size[0] *
            inner] * Linv->data[inner + Linv->size[0] * coffset];
        }
      }
    }
  } else {
    m = a->size[0];
    inner = a->size[1];
    n = Linv->size[1];
    i9 = Hinv->size[0] * Hinv->size[1];
    Hinv->size[0] = a->size[0];
    Hinv->size[1] = Linv->size[1];
    emxEnsureCapacity_real_T(Hinv, i9);
    for (loop_ub = 0; loop_ub < n; loop_ub++) {
      coffset = loop_ub * m;
      boffset = loop_ub * inner;
      for (i = 0; i < m; i++) {
        Hinv->data[coffset + i] = 0.0;
      }

      for (k = 0; k < inner; k++) {
        aoffset = k * m;
        temp = Linv->data[boffset + k];
        for (i = 0; i < m; i++) {
          i9 = coffset + i;
          Hinv->data[i9] += temp * a->data[aoffset + i];
        }
      }
    }
  }

  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    m = A->size[1];
  } else {
    m = 0;
  }

  if ((m == 0) || ((A->size[0] != 0) && (A->size[1] != 0))) {
    inner = A->size[0];
  } else {
    inner = 0;
  }

  emxInitMatrix_cell_wrap_0(reshapes);
  reshapes[1].f1->size[0] = 0;
  reshapes[1].f1->size[1] = m;
  emxInit_int16_T(&iA1, 1);
  i9 = iA1->size[0];
  iA1->size[0] = iA0->size[0];
  emxEnsureCapacity_int16_T(iA1, i9);
  loop_ub = iA0->size[0];
  for (i9 = 0; i9 < loop_ub; i9++) {
    iA1->data[i9] = iA0->data[i9];
  }

  i9 = a->size[0] * a->size[1];
  a->size[0] = inner + reshapes[1].f1->size[0];
  a->size[1] = m;
  emxEnsureCapacity_real_T(a, i9);
  for (i9 = 0; i9 < m; i9++) {
    for (coffset = 0; coffset < inner; coffset++) {
      a->data[coffset + a->size[0] * i9] = A->data[coffset + inner * i9];
    }
  }

  loop_ub = reshapes[1].f1->size[1];
  for (i9 = 0; i9 < loop_ub; i9++) {
    n = reshapes[1].f1->size[0];
    for (coffset = 0; coffset < n; coffset++) {
      a->data[(coffset + inner) + a->size[0] * i9] = reshapes[1].f1->
        data[coffset + reshapes[1].f1->size[0] * i9];
    }
  }

  emxFreeMatrix_cell_wrap_0(reshapes);
  emxInit_real_T(&lam, 1);
  i9 = A->size[0];
  if (i9 > 32767) {
    i9 = 32767;
  } else {
    if (i9 < -32768) {
      i9 = -32768;
    }
  }

  coffset = Linv->size[0];
  if (coffset > 32767) {
    coffset = 32767;
  } else {
    if (coffset < -32768) {
      coffset = -32768;
    }
  }

  qpkwik(Linv, Hinv, f, a, b, iA1, (short)i9, (short)coffset, x, lam, status);
  emxFree_real_T(&a);
  emxFree_int16_T(&iA1);
  emxFree_real_T(&lam);
  emxFree_real_T(&Hinv);
}

/* End of code generation (mpcqpsolver.cpp) */
