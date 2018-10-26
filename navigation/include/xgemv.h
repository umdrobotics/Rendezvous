/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xgemv.h
 *
 * Code generation for function 'xgemv'
 *
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void xgemv(int m, int n, const emxArray_real_T *A, int ia0, int lda,
                  const emxArray_real_T *x, int ix0, emxArray_real_T *y);

#endif

/* End of code generation (xgemv.h) */
