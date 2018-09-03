/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * solveQP.cpp
 *
 * Code generation for function 'solveQP'
 *
 */

/* Include files */
#include <cmath>
#include <string.h>
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/mpcqpsolver.h"
#include "navigation/solveQP_emxutil.h"
#include "navigation/inv.h"
#include "navigation/mpower.h"
#include "navigation/eye.h"

/* Function Definitions */
void solveQP(const double xk[4], const emxArray_real_T *rp, double x_data[], int
             x_size[1])
{
  double Ap[192];
  double Ap0[16];
  int jj;
  int i0;
  double Bp[480];
  int jmax;
  int i1;
  signed char Cv[1152];
  double Bpj[40];
  double b_Ap0[16];
  int loop_ub;
  static const double b[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.025,
    0.0, 0.9994, 0.0, 0.0, 0.025, 0.0, 0.9994 };

  double Bpi[8];
  static const double B[8] = { -0.0031, 0.0, -0.2451, 0.0, 0.0, -0.0031, 0.0,
    -0.2451 };

  double G[680];
  static const signed char iv0[100] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1 };

  int j;
  static const signed char iv1[100] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 };

  short b_Cv[1152];
  double S[68];
  int info;
  double b_b[8];
  double ajj;
  double c_Cv[96];
  double d_Cv[96];
  double A_data[100];
  double R[100];
  double b_Bp[480];
  boolean_T exitg1;
  static const double c_b[2304] = { 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.99 };

  double L_data[100];
  static const double b_R[100] = { 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.010000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.010000000000000009 };

  int ix;
  int iy;
  int L_size[2];
  double c;
  emxArray_real_T *b_rp;
  double b_xk[48];
  double c_xk[48];
  double d_xk[10];
  double e_xk[10];

  /*      H = [1 -1; -1 2]; */
  /*      f = [-2; -6]; */
  /*      A = [1 1; -1 2; 2 1]; */
  /*      b = [2; 2; 3]; */
  /*      lb = [0; 0]; */
  /*      [L,p] = chol(H,'lower'); */
  /*      Linv = inv(L); */
  /*      opt = mpcqpsolverOptions; */
  /*      iA0 = false(size(b)); */
  /*      Aeq = []; */
  /*      beq = zeros(0,1); */
  /*  %     [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb); */
  /*      [x,status] = mpcqpsolver(Linv,f,A,b,Aeq,beq,iA0,opt); */
  /*  dt = 0.1;           % seconds */
  /*   */
  /*  A = [1    0   dt  0;   */
  /*       0    1   0   dt; */
  /*       0    0   1   0; */
  /*       0    0   0   1 ]; */
  /*   */
  /*  B = [dt^2/2     0; */
  /*       0          dt^2/2; */
  /*       dt         0; */
  /*       0          dt]; */
  /*  rp = repmat([100,100,0,0]', P ,1); */
  /*  xk = [0,0,0,0]'; */
  /*  Build Ap */
  memset(&Ap[0], 0, 192U * sizeof(double));
  eye(Ap0);
  for (jj = 0; jj < 12; jj++) {
    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        b_Ap0[i0 + (i1 << 2)] = 0.0;
        for (loop_ub = 0; loop_ub < 4; loop_ub++) {
          b_Ap0[i0 + (i1 << 2)] += Ap0[i0 + (loop_ub << 2)] * b[loop_ub + (i1 <<
            2)];
        }
      }
    }

    jmax = jj << 2;
    for (i0 = 0; i0 < 4; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        Ap0[i1 + (i0 << 2)] = b_Ap0[i1 + (i0 << 2)];
        Ap[(i1 + jmax) + 48 * i0] = Ap0[i1 + (i0 << 2)];
      }
    }
  }

  /*  Build Bp */
  memset(&Bp[0], 0, 480U * sizeof(double));
  for (jj = 0; jj < 12; jj++) {
    memset(&Bpj[0], 0, 40U * sizeof(double));
    if (1 + jj < 5) {
      memcpy(&Bpi[0], &B[0], sizeof(double) << 3);
    } else {
      mpower(b, (1.0 + (double)jj) - 5.0, Ap0);
      for (i0 = 0; i0 < 4; i0++) {
        for (i1 = 0; i1 < 2; i1++) {
          Bpi[i0 + (i1 << 2)] = 0.0;
          for (loop_ub = 0; loop_ub < 4; loop_ub++) {
            Bpi[i0 + (i1 << 2)] += Ap0[i0 + (loop_ub << 2)] * B[loop_ub + (i1 <<
              2)];
          }
        }
      }
    }

    if (5 < 1 + jj) {
      i0 = 4;
    } else {
      i0 = jj;
    }

    i1 = (int)((1.0 + (-1.0 - ((double)i0 + 1.0))) / -1.0);
    for (j = 0; j < i1; j++) {
      jmax = (i0 - j) << 1;
      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        for (info = 0; info < 4; info++) {
          Bpj[info + ((loop_ub + jmax) << 2)] = Bpi[info + (loop_ub << 2)];
        }
      }

      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        for (info = 0; info < 2; info++) {
          b_b[loop_ub + (info << 2)] = 0.0;
          for (jmax = 0; jmax < 4; jmax++) {
            b_b[loop_ub + (info << 2)] += b[loop_ub + (jmax << 2)] * Bpi[jmax +
              (info << 2)];
          }
        }
      }

      for (loop_ub = 0; loop_ub < 2; loop_ub++) {
        for (info = 0; info < 4; info++) {
          Bpi[info + (loop_ub << 2)] = b_b[info + (loop_ub << 2)];
        }
      }
    }

    jmax = jj << 2;
    for (i0 = 0; i0 < 10; i0++) {
      for (i1 = 0; i1 < 4; i1++) {
        Bp[(i1 + jmax) + 48 * i0] = Bpj[i1 + (i0 << 2)];
      }
    }

    /*                  disp(Bp); */
  }

  memset(&Cv[0], 0, 1152U * sizeof(signed char));
  for (jj = 0; jj < 12; jj++) {
    Cv[(jj << 1) + 24 * ((jj << 2) + 2)] = 1;
    Cv[((jj << 1) + 24 * ((jj << 2) + 3)) + 1] = 1;
  }

  memset(&G[0], 0, 680U * sizeof(double));
  for (i0 = 0; i0 < 10; i0++) {
    for (i1 = 0; i1 < 10; i1++) {
      G[i1 + 68 * i0] = iv0[i1 + 10 * i0];
      G[(i1 + 68 * i0) + 10] = iv1[i1 + 10 * i0];
    }
  }

  for (i0 = 0; i0 < 24; i0++) {
    for (i1 = 0; i1 < 10; i1++) {
      G[(i0 + 68 * i1) + 20] = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        G[(i0 + 68 * i1) + 20] += (double)Cv[i0 + 24 * loop_ub] * Bp[loop_ub +
          48 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 48; i0++) {
    for (i1 = 0; i1 < 24; i1++) {
      b_Cv[i1 + 24 * i0] = (short)-Cv[i1 + 24 * i0];
    }
  }

  for (i0 = 0; i0 < 24; i0++) {
    for (i1 = 0; i1 < 10; i1++) {
      G[(i0 + 68 * i1) + 44] = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        G[(i0 + 68 * i1) + 44] += (double)b_Cv[i0 + 24 * loop_ub] * Bp[loop_ub +
          48 * i1];
      }
    }
  }

  for (i0 = 0; i0 < 680; i0++) {
    G[i0] = -G[i0];
  }

  memset(&S[0], 0, 68U * sizeof(double));
  for (jj = 0; jj < 20; jj++) {
    S[jj] = 20.0;
  }

  for (i0 = 0; i0 < 24; i0++) {
    ajj = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      c_Cv[i0 + 24 * i1] = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        c_Cv[i0 + 24 * i1] += (double)Cv[i0 + 24 * loop_ub] * Ap[loop_ub + 48 *
          i1];
      }

      ajj += c_Cv[i0 + 24 * i1] * xk[i1];
      d_Cv[i0 + 24 * i1] = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        d_Cv[i0 + 24 * i1] += (double)Cv[i0 + 24 * loop_ub] * Ap[loop_ub + 48 *
          i1];
      }
    }

    S[20 + i0] = 17.0 - ajj;
    ajj = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      ajj += d_Cv[i0 + 24 * i1] * xk[i1];
    }

    S[44 + i0] = 17.0 + ajj;
  }

  for (i0 = 0; i0 < 68; i0++) {
    S[i0] = -S[i0];
  }

  /*  G = eye(2*M, 2*M); */
  /*  G = [G ; -eye(2*M, 2*M)]; */
  /*  G = -G; */
  /*  S = ones(4*M,1)*20; */
  /*  S = -S; */
  /*  G = Cv*Bp; */
  /*  G = [G ; -Cv*Bp]; */
  /*   */
  /*  S = ones(2*P,1)*17 - Cv*Ap*xk; */
  /*  S = [S; ones(2*P,1)*17 + Cv*Ap*xk]; */
  /*  ub = ones(2*M,1)*20; */
  /*  lb = ones(2*M,1)*(-20); */
  for (i0 = 0; i0 < 10; i0++) {
    for (i1 = 0; i1 < 48; i1++) {
      b_Bp[i0 + 10 * i1] = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        b_Bp[i0 + 10 * i1] += Bp[loop_ub + 48 * i0] * c_b[loop_ub + 48 * i1];
      }
    }

    for (i1 = 0; i1 < 10; i1++) {
      ajj = 0.0;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        ajj += b_Bp[i0 + 10 * loop_ub] * Bp[loop_ub + 48 * i1];
      }

      R[i0 + 10 * i1] = b_R[i0 + 10 * i1] + ajj;
    }
  }

  for (i0 = 0; i0 < 10; i0++) {
    memcpy(&A_data[i0 * 10], &R[i0 * 10], 10U * sizeof(double));
  }

  info = 0;
  j = 1;
  exitg1 = false;
  while ((!exitg1) && (j <= 10)) {
    jj = (j + (j - 1) * 10) - 1;
    ajj = 0.0;
    if (!(j - 1 < 1)) {
      ix = j;
      iy = j;
      for (jmax = 1; jmax < j; jmax++) {
        ajj += A_data[ix - 1] * A_data[iy - 1];
        ix += 10;
        iy += 10;
      }
    }

    ajj = A_data[jj] - ajj;
    if (ajj > 0.0) {
      ajj = std::sqrt(ajj);
      A_data[jj] = ajj;
      if (j < 10) {
        if (j - 1 != 0) {
          ix = j;
          i0 = (j + 10 * (j - 2)) + 1;
          for (jmax = j + 1; jmax <= i0; jmax += 10) {
            c = -A_data[ix - 1];
            iy = jj + 1;
            i1 = (jmax - j) + 9;
            for (loop_ub = jmax; loop_ub <= i1; loop_ub++) {
              A_data[iy] += A_data[loop_ub - 1] * c;
              iy++;
            }

            ix += 10;
          }
        }

        ajj = 1.0 / ajj;
        memcpy(&L_data[0], &A_data[0], 100U * sizeof(double));
        i0 = jj - j;
        for (jmax = jj + 1; jmax < i0 + 11; jmax++) {
          L_data[jmax] *= ajj;
        }

        memcpy(&A_data[0], &L_data[0], (unsigned int)(100 * (int)sizeof(double)));
      }

      j++;
    } else {
      A_data[jj] = ajj;
      info = j;
      exitg1 = true;
    }
  }

  memcpy(&L_data[0], &A_data[0], 100U * sizeof(double));
  if (info == 0) {
    jmax = 10;
  } else {
    jmax = info - 1;
  }

  for (j = 1; j < jmax; j++) {
    for (jj = 1; jj <= j; jj++) {
      L_data[(jj + 10 * j) - 1] = 0.0;
    }
  }

  if (1 > jmax) {
    loop_ub = 0;
    jmax = 0;
  } else {
    loop_ub = jmax;
  }

  for (i0 = 0; i0 < jmax; i0++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      R[i1 + loop_ub * i0] = L_data[i1 + 10 * i0];
    }
  }

  L_size[0] = loop_ub;
  L_size[1] = jmax;
  for (i0 = 0; i0 < jmax; i0++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      L_data[i1 + loop_ub * i0] = R[i1 + loop_ub * i0];
    }
  }

  emxInit_real_T(&b_rp, 2);

  /*      [x,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[],lb); */
  inv(L_data, L_size);
  i0 = b_rp->size[0] * b_rp->size[1];
  b_rp->size[0] = 1;
  b_rp->size[1] = rp->size[0];
  emxEnsureCapacity_real_T(b_rp, i0);
  loop_ub = rp->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b_rp->data[b_rp->size[0] * i0] = rp->data[i0];
  }

  for (i0 = 0; i0 < 48; i0++) {
    ajj = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      ajj += xk[i1] * Ap[i0 + 48 * i1];
    }

    c_xk[i0] = ajj - b_rp->data[i0];
  }

  emxFree_real_T(&b_rp);
  for (i0 = 0; i0 < 48; i0++) {
    b_xk[i0] = 0.0;
    for (i1 = 0; i1 < 48; i1++) {
      b_xk[i0] += c_xk[i1] * c_b[i1 + 48 * i0];
    }
  }

  for (i0 = 0; i0 < 10; i0++) {
    e_xk[i0] = 0.0;
    for (i1 = 0; i1 < 48; i1++) {
      e_xk[i0] += b_xk[i1] * Bp[i1 + 48 * i0];
    }

    d_xk[i0] = e_xk[i0];
  }

  mpcqpsolver(L_data, L_size, d_xk, G, S, x_data, x_size, &ajj);
}

/* End of code generation (solveQP.cpp) */
