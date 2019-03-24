//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 29-Oct-2018 21:02:18
//

// Include Files
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "inv.h"
#include "solveQP_emxutil.h"

// Function Definitions

//
// Arguments    : const double x[16]
//                double y[16]
// Return Type  : void
//
void inv(const double x[16], double y[16])
{
  int i3;
  double b_x[16];
  int j;
  signed char ipiv[4];
  int c;
  int iy;
  int k;
  signed char p[4];
  int ix;
  double smax;
  double s;
  int jy;
  int i;
  for (i3 = 0; i3 < 16; i3++) {
    y[i3] = 0.0;
    b_x[i3] = x[i3];
  }

  for (i3 = 0; i3 < 4; i3++) {
    ipiv[i3] = (signed char)(1 + i3);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    iy = 0;
    ix = c;
    smax = std::abs(b_x[c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = std::abs(b_x[ix]);
      if (s > smax) {
        iy = k - 1;
        smax = s;
      }
    }

    if (b_x[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (k = 0; k < 4; k++) {
          smax = b_x[ix];
          b_x[ix] = b_x[iy];
          b_x[iy] = smax;
          ix += 4;
          iy += 4;
        }
      }

      i3 = (c - j) + 4;
      for (i = c + 1; i + 1 <= i3; i++) {
        b_x[i] /= b_x[c];
      }
    }

    iy = c + 5;
    jy = c + 4;
    for (i = 1; i <= 3 - j; i++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0) {
        ix = c + 1;
        i3 = (iy - j) + 3;
        for (k = iy; k + 1 <= i3; k++) {
          b_x[k] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  for (i3 = 0; i3 < 4; i3++) {
    p[i3] = (signed char)(1 + i3);
  }

  for (k = 0; k < 3; k++) {
    if (ipiv[k] > 1 + k) {
      iy = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (signed char)iy;
    }
  }

  for (k = 0; k < 4; k++) {
    c = p[k] - 1;
    y[k + ((p[k] - 1) << 2)] = 1.0;
    for (j = k; j + 1 < 5; j++) {
      if (y[j + (c << 2)] != 0.0) {
        for (i = j + 1; i + 1 < 5; i++) {
          y[i + (c << 2)] -= y[j + (c << 2)] * b_x[i + (j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    iy = j << 2;
    for (k = 3; k >= 0; k--) {
      jy = k << 2;
      if (y[k + iy] != 0.0) {
        y[k + iy] /= b_x[k + jy];
        for (i = 0; i + 1 <= k; i++) {
          y[i + iy] -= y[k + iy] * b_x[i + jy];
        }
      }
    }
  }
}

//
// Arguments    : const emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int n;
  int i5;
  int yk;
  emxArray_real_T *b_x;
  int b_n;
  emxArray_int32_T *ipiv;
  int k;
  int u1;
  emxArray_int32_T *p;
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int jy;
  double s;
  int ijA;
  n = x->size[0];
  i5 = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, i5);
  yk = x->size[0] * x->size[1];
  for (i5 = 0; i5 < yk; i5++) {
    y->data[i5] = 0.0;
  }

  emxInit_real_T(&b_x, 2);
  i5 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = x->size[1];
  emxEnsureCapacity_real_T(b_x, i5);
  yk = x->size[0] * x->size[1];
  for (i5 = 0; i5 < yk; i5++) {
    b_x->data[i5] = x->data[i5];
  }

  yk = x->size[0];
  if (yk < 1) {
    b_n = 0;
  } else {
    b_n = yk;
  }

  emxInit_int32_T1(&ipiv, 2);
  i5 = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T1(ipiv, i5);
  if (b_n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv->data[k - 1] = yk;
    }
  }

  if (x->size[0] < 1) {
    b_n = 0;
  } else {
    yk = x->size[0] - 1;
    u1 = x->size[0];
    if (yk < u1) {
      u1 = yk;
    }

    for (j = 0; j + 1 <= u1; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          ix = c;
          smax = std::abs(b_x->data[c]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = std::abs(b_x->data[ix]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (b_x->data[c + yk] != 0.0) {
        if (yk != 0) {
          ipiv->data[j] = (j + yk) + 1;
          ix = j;
          yk += j;
          for (k = 1; k <= n; k++) {
            smax = b_x->data[ix];
            b_x->data[ix] = b_x->data[yk];
            b_x->data[yk] = smax;
            ix += n;
            yk += n;
          }
        }

        i5 = c + mmj;
        for (jy = c + 1; jy + 1 <= i5; jy++) {
          b_x->data[jy] /= b_x->data[c];
        }
      }

      yk = n - j;
      b_n = (c + n) + 1;
      jy = c + n;
      for (k = 1; k < yk; k++) {
        smax = b_x->data[jy];
        if (b_x->data[jy] != 0.0) {
          ix = c + 1;
          i5 = mmj + b_n;
          for (ijA = b_n; ijA + 1 < i5; ijA++) {
            b_x->data[ijA] += b_x->data[ix] * -smax;
            ix++;
          }
        }

        jy += n;
        b_n += n;
      }
    }

    b_n = x->size[0];
  }

  emxInit_int32_T1(&p, 2);
  i5 = p->size[0] * p->size[1];
  p->size[0] = 1;
  p->size[1] = b_n;
  emxEnsureCapacity_int32_T1(p, i5);
  if (b_n > 0) {
    p->data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      p->data[k - 1] = yk;
    }
  }

  for (k = 0; k < ipiv->size[1]; k++) {
    if (ipiv->data[k] > 1 + k) {
      yk = p->data[ipiv->data[k] - 1];
      p->data[ipiv->data[k] - 1] = p->data[k];
      p->data[k] = yk;
    }
  }

  emxFree_int32_T(&ipiv);
  for (k = 0; k + 1 <= n; k++) {
    c = p->data[k] - 1;
    y->data[k + y->size[0] * (p->data[k] - 1)] = 1.0;
    for (j = k; j + 1 <= n; j++) {
      if (y->data[j + y->size[0] * c] != 0.0) {
        for (jy = j + 1; jy + 1 <= n; jy++) {
          y->data[jy + y->size[0] * c] -= y->data[j + y->size[0] * c] *
            b_x->data[jy + b_x->size[0] * j];
        }
      }
    }
  }

  emxFree_int32_T(&p);
  if ((x->size[0] == 0) || ((y->size[0] == 0) || (y->size[1] == 0))) {
  } else {
    for (j = 1; j <= n; j++) {
      yk = n * (j - 1) - 1;
      for (k = n; k > 0; k--) {
        b_n = n * (k - 1) - 1;
        if (y->data[k + yk] != 0.0) {
          y->data[k + yk] /= b_x->data[k + b_n];
          for (jy = 1; jy < k; jy++) {
            y->data[jy + yk] -= y->data[k + yk] * b_x->data[jy + b_n];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_x);
}

//
// File trailer for inv.cpp
//
// [EOF]
//
