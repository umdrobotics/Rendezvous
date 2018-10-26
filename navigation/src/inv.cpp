/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inv.cpp
 *
 * Code generation for function 'inv'
 *
 */

/* Include files */
#include <cmath>
#include "rt_nonfinite.h"
#include "solveQP.h"
#include "inv.h"
#include "solveQP_emxutil.h"

/* Function Definitions */
void inv(const double x[16], double y[16])
{
  int i5;
  signed char ipiv[4];
  double b_x[16];
  int j;
  signed char p[4];
  int b;
  int jj;
  int jp1j;
  int n;
  int jy;
  int ix;
  int iy;
  double smax;
  double s;
  int i;
  for (i5 = 0; i5 < 16; i5++) {
    y[i5] = 0.0;
    b_x[i5] = x[i5];
  }

  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  for (j = 0; j < 3; j++) {
    b = j * 5;
    jj = j * 5;
    jp1j = b + 2;
    n = 4 - j;
    jy = 0;
    ix = b;
    smax = std::abs(b_x[b]);
    for (iy = 2; iy <= n; iy++) {
      ix++;
      s = std::abs(b_x[ix]);
      if (s > smax) {
        jy = iy - 1;
        smax = s;
      }
    }

    if (b_x[jj + jy] != 0.0) {
      if (jy != 0) {
        iy = j + jy;
        ipiv[j] = (signed char)(iy + 1);
        smax = b_x[j];
        b_x[j] = b_x[iy];
        b_x[iy] = smax;
        ix = j + 4;
        iy += 4;
        smax = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = smax;
        ix += 4;
        iy += 4;
        smax = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = smax;
        ix += 4;
        iy += 4;
        smax = b_x[ix];
        b_x[ix] = b_x[iy];
        b_x[iy] = smax;
      }

      i5 = jj - j;
      for (i = jp1j; i <= i5 + 4; i++) {
        b_x[i - 1] /= b_x[jj];
      }
    }

    n = 2 - j;
    jy = b + 4;
    iy = jj + 5;
    for (b = 0; b <= n; b++) {
      smax = b_x[jy];
      if (b_x[jy] != 0.0) {
        ix = jj + 1;
        i5 = iy + 1;
        i = (iy - j) + 3;
        for (jp1j = i5; jp1j <= i; jp1j++) {
          b_x[jp1j - 1] += b_x[ix] * -smax;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > 1) {
    jy = ipiv[0] - 1;
    iy = p[jy];
    p[jy] = 1;
    p[0] = (signed char)iy;
  }

  if (ipiv[1] > 2) {
    jy = ipiv[1] - 1;
    iy = p[jy];
    p[jy] = p[1];
    p[1] = (signed char)iy;
  }

  if (ipiv[2] > 3) {
    jy = ipiv[2] - 1;
    iy = p[jy];
    p[jy] = p[2];
    p[2] = (signed char)iy;
  }

  jy = p[0] - 1;
  b = jy << 2;
  y[b] = 1.0;
  for (j = 1; j < 5; j++) {
    if (y[(j + b) - 1] != 0.0) {
      i5 = j + 1;
      for (i = i5; i < 5; i++) {
        iy = (i + b) - 1;
        y[iy] -= y[(j + (jy << 2)) - 1] * b_x[(i + ((j - 1) << 2)) - 1];
      }
    }
  }

  jy = p[1] - 1;
  b = jy << 2;
  y[1 + b] = 1.0;
  for (j = 2; j < 5; j++) {
    if (y[(j + b) - 1] != 0.0) {
      i5 = j + 1;
      for (i = i5; i < 5; i++) {
        iy = (i + b) - 1;
        y[iy] -= y[(j + (jy << 2)) - 1] * b_x[(i + ((j - 1) << 2)) - 1];
      }
    }
  }

  jy = p[2] - 1;
  b = jy << 2;
  y[2 + b] = 1.0;
  for (j = 3; j < 5; j++) {
    if (y[(j + b) - 1] != 0.0) {
      i5 = j + 1;
      for (i = i5; i < 5; i++) {
        iy = b + 3;
        y[iy] -= y[(j + (jy << 2)) - 1] * b_x[((j - 1) << 2) + 3];
      }
    }
  }

  y[3 + ((p[3] - 1) << 2)] = 1.0;
  for (j = 0; j < 4; j++) {
    jy = j << 2;
    smax = y[3 + jy];
    if (smax != 0.0) {
      y[3 + jy] = smax / b_x[15];
      for (i = 0; i < 3; i++) {
        b = i + jy;
        y[b] -= y[3 + jy] * b_x[i + 12];
      }
    }

    smax = y[2 + jy];
    if (smax != 0.0) {
      y[2 + jy] = smax / b_x[10];
      for (i = 0; i < 2; i++) {
        y[i + jy] -= y[2 + jy] * b_x[i + 8];
      }
    }

    smax = y[1 + jy];
    if (smax != 0.0) {
      y[1 + jy] = smax / b_x[5];
      for (i = 0; i < 1; i++) {
        y[jy] -= y[1 + jy] * b_x[4];
      }
    }

    if (y[jy] != 0.0) {
      y[jy] /= b_x[0];
    }
  }
}

void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int n;
  int i7;
  int yk;
  emxArray_real_T *b_x;
  int b_n;
  emxArray_int32_T *ipiv;
  int ldap1;
  int k;
  int u1;
  emxArray_int32_T *p;
  int j;
  int mmj_tmp;
  int jj;
  int jp1j;
  int ix;
  double smax;
  int i;
  double s;
  int i8;
  n = x->size[0];
  i7 = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, i7);
  yk = x->size[0] * x->size[1];
  for (i7 = 0; i7 < yk; i7++) {
    y->data[i7] = 0.0;
  }

  emxInit_real_T(&b_x, 2);
  i7 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = x->size[1];
  emxEnsureCapacity_real_T(b_x, i7);
  yk = x->size[0] * x->size[1];
  for (i7 = 0; i7 < yk; i7++) {
    b_x->data[i7] = x->data[i7];
  }

  b_n = x->size[0];
  if (b_n < 1) {
    b_n = 0;
  }

  emxInit_int32_T(&ipiv, 2);
  i7 = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T(ipiv, i7);
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
    ldap1 = x->size[0] + 1;
    yk = x->size[0] - 1;
    u1 = x->size[0];
    if (yk < u1) {
      u1 = yk;
    }

    for (j = 0; j < u1; j++) {
      mmj_tmp = n - j;
      b_n = j * (n + 1);
      jj = j * ldap1;
      jp1j = b_n + 2;
      if (mmj_tmp < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj_tmp > 1) {
          ix = b_n;
          smax = std::abs(b_x->data[b_n]);
          for (k = 2; k <= mmj_tmp; k++) {
            ix++;
            s = std::abs(b_x->data[ix]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (b_x->data[jj + yk] != 0.0) {
        if (yk != 0) {
          yk += j;
          ipiv->data[j] = yk + 1;
          ix = j;
          for (k = 0; k < n; k++) {
            smax = b_x->data[ix];
            b_x->data[ix] = b_x->data[yk];
            b_x->data[yk] = smax;
            ix += n;
            yk += n;
          }
        }

        i7 = jj + mmj_tmp;
        for (i = jp1j; i <= i7; i++) {
          b_x->data[i - 1] /= b_x->data[jj];
        }
      }

      yk = b_n + n;
      b_n = jj + ldap1;
      for (jp1j = 0; jp1j <= mmj_tmp - 2; jp1j++) {
        smax = b_x->data[yk];
        if (b_x->data[yk] != 0.0) {
          ix = jj + 1;
          i7 = b_n + 1;
          i8 = mmj_tmp + b_n;
          for (i = i7; i < i8; i++) {
            b_x->data[i - 1] += b_x->data[ix] * -smax;
            ix++;
          }
        }

        yk += n;
        b_n += n;
      }
    }

    b_n = x->size[0];
  }

  emxInit_int32_T(&p, 2);
  i7 = p->size[0] * p->size[1];
  p->size[0] = 1;
  p->size[1] = b_n;
  emxEnsureCapacity_int32_T(p, i7);
  if (b_n > 0) {
    p->data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      p->data[k - 1] = yk;
    }
  }

  i7 = ipiv->size[1];
  for (k = 0; k < i7; k++) {
    if (ipiv->data[k] > 1 + k) {
      yk = p->data[ipiv->data[k] - 1];
      p->data[ipiv->data[k] - 1] = p->data[k];
      p->data[k] = yk;
    }
  }

  emxFree_int32_T(&ipiv);
  for (k = 0; k < n; k++) {
    yk = p->data[k] - 1;
    y->data[k + y->size[0] * (p->data[k] - 1)] = 1.0;
    for (j = k + 1; j <= n; j++) {
      if (y->data[(j + y->size[0] * yk) - 1] != 0.0) {
        i7 = j + 1;
        for (i = i7; i <= n; i++) {
          y->data[(i + y->size[0] * yk) - 1] -= y->data[(j + y->size[0] * yk) -
            1] * b_x->data[(i + b_x->size[0] * (j - 1)) - 1];
        }
      }
    }
  }

  emxFree_int32_T(&p);
  if ((x->size[0] == 0) || ((y->size[0] == 0) || (y->size[1] == 0))) {
  } else {
    for (j = 0; j < n; j++) {
      yk = n * j - 1;
      for (k = n; k >= 1; k--) {
        b_n = n * (k - 1) - 1;
        i7 = k + yk;
        if (y->data[i7] != 0.0) {
          y->data[i7] /= b_x->data[k + b_n];
          for (i = 0; i <= k - 2; i++) {
            i8 = (i + yk) + 1;
            y->data[i8] -= y->data[i7] * b_x->data[(i + b_n) + 1];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_x);
}

/* End of code generation (inv.cpp) */
