/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_StaticForceTransform_FtsToTcp_api.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

#ifndef _CODER_STATICFORCETRANSFORM_FTSTOTCP_API_H
#define _CODER_STATICFORCETRANSFORM_FTSTOTCP_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void StaticForceTransform_FtsToTcp(real_T Fx, real_T Fy, real_T Fz, real_T Tx,
                                   real_T Ty, real_T Tz, real_T dx, real_T dy,
                                   real_T dz, real_T rx, real_T ry, real_T rz,
                                   real_T FTtcp_sensor[6]);

void StaticForceTransform_FtsToTcp_atexit(void);

void StaticForceTransform_FtsToTcp_initialize(void);

void StaticForceTransform_FtsToTcp_terminate(void);

void StaticForceTransform_FtsToTcp_xil_shutdown(void);

void StaticForceTransform_FtsToTcp_xil_terminate(void);

void c_StaticForceTransform_FtsToTcp(const mxArray *const prhs[12],
                                     const mxArray **plhs);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_StaticForceTransform_FtsToTcp_api.h
 *
 * [EOF]
 */
