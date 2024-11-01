/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_StaticForceTransform_FtsToTcp_mex.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

/* Include Files */
#include "_coder_StaticForceTransform_FtsToTcp_mex.h"
#include "_coder_StaticForceTransform_FtsToTcp_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&StaticForceTransform_FtsToTcp_atexit);
  /* Module initialization. */
  StaticForceTransform_FtsToTcp_initialize();
  /* Dispatch the entry-point. */
  unsafe_StaticForceTransform_FtsToTcp_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  StaticForceTransform_FtsToTcp_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[12]
 * Return Type  : void
 */
void unsafe_StaticForceTransform_FtsToTcp_mexFunction(int32_T nlhs,
                                                      mxArray *plhs[1],
                                                      int32_T nrhs,
                                                      const mxArray *prhs[12])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 12) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 12, 4,
                        29, "StaticForceTransform_FtsToTcp");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 29,
                        "StaticForceTransform_FtsToTcp");
  }
  /* Call the function. */
  c_StaticForceTransform_FtsToTcp(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * File trailer for _coder_StaticForceTransform_FtsToTcp_mex.c
 *
 * [EOF]
 */
