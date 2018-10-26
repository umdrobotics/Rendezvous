/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xger.h
 *
 * Code generation for function 'xger'
 *
 */

#ifndef XGER_H
#define XGER_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void xger(int m, int n, double alpha1, int ix0, const emxArray_real_T *y,
                 emxArray_real_T *A, int ia0, int lda);

#endif

/* End of code generation (xger.h) */
