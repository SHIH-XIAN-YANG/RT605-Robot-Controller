/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: MatInv4.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 24-Aug-2022 14:21:07
 */

/* Include Files */
#include "MatInv4.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double mat[16]
 *                double matInv[16]
 * Return Type  : void
 */
void MatInv4(const double mat[16], double matInv[16])
{
  double x[16];
  double s;
  double smax;
  int b_i;
  int b_tmp;
  int i;
  int i1;
  int j;
  int jA;
  int jp1j;
  int k;
  int kAcol;
  int mmj_tmp;
  signed char ipiv[4];
  signed char p[4];
  for (i = 0; i < 16; i++) {
    matInv[i] = 0.0;
    x[i] = mat[i];
  }
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  for (j = 0; j < 3; j++) {
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jA = 4 - j;
    kAcol = 0;
    smax = fabs(x[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = fabs(x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (x[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = (signed char)(jA + 1);
        smax = x[j];
        x[j] = x[jA];
        x[jA] = smax;
        smax = x[j + 4];
        x[j + 4] = x[jA + 4];
        x[jA + 4] = smax;
        smax = x[j + 8];
        x[j + 8] = x[jA + 8];
        x[jA + 8] = smax;
        smax = x[j + 12];
        x[j + 12] = x[jA + 12];
        x[jA + 12] = smax;
      }
      i = (b_tmp - j) + 4;
      for (b_i = jp1j; b_i <= i; b_i++) {
        x[b_i - 1] /= x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = x[(b_tmp + (kAcol << 2)) + 4];
      if (smax != 0.0) {
        i = jA + 6;
        i1 = (jA - j) + 8;
        for (jp1j = i; jp1j <= i1; jp1j++) {
          x[jp1j - 1] += x[((b_tmp + jp1j) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
  }
  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > 1) {
    jA = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = 1;
    p[0] = (signed char)jA;
  }
  if (ipiv[1] > 2) {
    jA = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = (signed char)jA;
  }
  if (ipiv[2] > 3) {
    jA = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = (signed char)jA;
  }
  for (k = 0; k < 4; k++) {
    jp1j = (p[k] - 1) << 2;
    matInv[k + jp1j] = 1.0;
    for (j = k + 1; j < 5; j++) {
      i = (j + jp1j) - 1;
      if (matInv[i] != 0.0) {
        i1 = j + 1;
        for (b_i = i1; b_i < 5; b_i++) {
          jA = (b_i + jp1j) - 1;
          matInv[jA] -= matInv[i] * x[(b_i + ((j - 1) << 2)) - 1];
        }
      }
    }
  }
  for (j = 0; j < 4; j++) {
    jA = j << 2;
    for (k = 3; k >= 0; k--) {
      kAcol = k << 2;
      i = k + jA;
      smax = matInv[i];
      if (smax != 0.0) {
        matInv[i] = smax / x[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          jp1j = b_i + jA;
          matInv[jp1j] -= matInv[i] * x[b_i + kAcol];
        }
      }
    }
  }
}

/*
 * File trailer for MatInv4.c
 *
 * [EOF]
 */
