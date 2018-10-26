/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qpkwik.h
 *
 * Code generation for function 'qpkwik'
 *
 */

#ifndef QPKWIK_H
#define QPKWIK_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void qpkwik(const emxArray_real_T *Linv, const emxArray_real_T *Hinv,
                   const emxArray_real_T *f, const emxArray_real_T *Ac, const
                   emxArray_real_T *b, emxArray_int16_T *iA, short m, short n,
                   emxArray_real_T *x, emxArray_real_T *lambda, double *status);

#endif

/* End of code generation (qpkwik.h) */
