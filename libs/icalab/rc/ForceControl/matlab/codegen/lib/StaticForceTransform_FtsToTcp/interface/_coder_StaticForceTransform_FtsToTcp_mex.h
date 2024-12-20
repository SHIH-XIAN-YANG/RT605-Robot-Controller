/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_StaticForceTransform_FtsToTcp_mex.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

#ifndef _CODER_STATICFORCETRANSFORM_FTSTOTCP_MEX_H
#define _CODER_STATICFORCETRANSFORM_FTSTOTCP_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_StaticForceTransform_FtsToTcp_mexFunction(int32_T nlhs,
                                                      mxArray *plhs[1],
                                                      int32_T nrhs,
                                                      const mxArray *prhs[12]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_StaticForceTransform_FtsToTcp_mex.h
 *
 * [EOF]
 */
