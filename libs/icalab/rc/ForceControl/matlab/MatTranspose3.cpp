//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MatTranspose.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 24-Aug-2022 14:29:36
//

// Include Files
#include "MatTranspose.h"

// Function Definitions
//
// Arguments    : const double mat[9]
//                double matT[9]
// Return Type  : void
//
void MatTranspose_3_3(const double mat[9], double matT[9])
{
  for (int i{0}; i < 3; i++) {
    matT[3 * i] = mat[i];
    matT[3 * i + 1] = mat[i + 3];
    matT[3 * i + 2] = mat[i + 6];
  }
}

//
// File trailer for MatTranspose.cpp
//
// [EOF]
//
