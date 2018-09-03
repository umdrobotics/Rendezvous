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
extern void mpcqpsolver(const double Linv_data[], const int Linv_size[2], const
  double f[10], const double A[680], const double b[68], double x_data[], int
  x_size[1], double *status);

#endif

/* End of code generation (mpcqpsolver.h) */
