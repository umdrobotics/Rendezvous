/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inv.h
 *
 * Code generation for function 'inv'
 *
 */

#ifndef INV_H
#define INV_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void inv(const double x[16], double y[16]);
extern void invNxN(const emxArray_real_T *x, emxArray_real_T *y);

#endif

/* End of code generation (inv.h) */
