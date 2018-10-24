/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * qr.h
 *
 * Code generation for function 'qr'
 *
 */

#ifndef QR_H
#define QR_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void qr(const double A_data[], const int A_size[2], double Q_data[], int
               Q_size[2], double R_data[], int R_size[2]);

#endif

/* End of code generation (qr.h) */
