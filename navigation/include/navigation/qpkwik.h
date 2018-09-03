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
extern void qpkwik(const double Linv_data[], const int Linv_size[2], const
                   double Hinv_data[], const int Hinv_size[2], const double f[10],
                   const double Ac[680], const double b[68], double x_data[],
                   int x_size[1], double lambda[68], double *status, short iA[68]);

#endif

/* End of code generation (qpkwik.h) */
