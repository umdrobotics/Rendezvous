/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveQP.h
 *
 * Code generation for function 'solveQP'
 *
 */

#ifndef SOLVEQP_H
#define SOLVEQP_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void solveQP(double P, double M, const double xk[4], const
                    emxArray_real_T *rp, double q, double k, double Qf, double
                    Qb, emxArray_real_T *x);

#endif

/* End of code generation (solveQP.h) */
