/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mpcqpsolver.h
 *
 * Code generation for function 'mpcqpsolver'
 *
 */

#ifndef MPCQPSOLVER_H
#define MPCQPSOLVER_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void mpcqpsolver(const emxArray_real_T *Linv, const emxArray_real_T *f,
  const emxArray_real_T *A, const emxArray_real_T *b, const emxArray_boolean_T
  *iA0, emxArray_real_T *x, double *status);

#endif

/* End of code generation (mpcqpsolver.h) */
