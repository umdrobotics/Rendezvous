/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eye.h
 *
 * Code generation for function 'eye'
 *
 */

#ifndef EYE_H
#define EYE_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "solveQP_types.h"

/* Function Declarations */
extern void b_eye(double b_I[16]);
extern void c_eye(double varargin_1, double varargin_2, emxArray_real_T *b_I);
extern void eye(double varargin_1, emxArray_real_T *b_I);

#endif

/* End of code generation (eye.h) */
