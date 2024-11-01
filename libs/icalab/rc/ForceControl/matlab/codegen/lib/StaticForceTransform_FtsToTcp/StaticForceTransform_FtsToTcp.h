/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: StaticForceTransform_FtsToTcp.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

#ifndef STATICFORCETRANSFORM_FTSTOTCP_H
#define STATICFORCETRANSFORM_FTSTOTCP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void StaticForceTransform_FtsToTcp(double Fx, double Fy, double Fz,
                                          double Tx, double Ty, double Tz,
                                          double dx, double dy, double dz,
                                          double rx, double ry, double rz,
                                          double FTtcp_sensor[6]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for StaticForceTransform_FtsToTcp.h
 *
 * [EOF]
 */
