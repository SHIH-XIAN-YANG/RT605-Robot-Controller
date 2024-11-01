/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_StaticForceTransform_FtsToTcp_api.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

/* Include Files */
#include "_coder_StaticForceTransform_FtsToTcp_api.h"
#include "_coder_StaticForceTransform_FtsToTcp_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131610U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "StaticForceTransform_FtsToTcp",                      /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *Fx,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u[6]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Fx
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *Fx,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(Fx), &thisId);
  emlrtDestroyArray(&Fx);
  return y;
}

/*
 * Arguments    : const real_T u[6]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[6])
{
  static const int32_T i = 0;
  static const int32_T i1 = 6;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void StaticForceTransform_FtsToTcp_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  StaticForceTransform_FtsToTcp_xil_terminate();
  StaticForceTransform_FtsToTcp_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void StaticForceTransform_FtsToTcp_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void StaticForceTransform_FtsToTcp_terminate(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : const mxArray * const prhs[12]
 *                const mxArray **plhs
 * Return Type  : void
 */
void c_StaticForceTransform_FtsToTcp(const mxArray *const prhs[12],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*FTtcp_sensor)[6];
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
  real_T dx;
  real_T dy;
  real_T dz;
  real_T rx;
  real_T ry;
  real_T rz;
  st.tls = emlrtRootTLSGlobal;
  FTtcp_sensor = (real_T(*)[6])mxMalloc(sizeof(real_T[6]));
  /* Marshall function inputs */
  Fx = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "Fx");
  Fy = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "Fy");
  Fz = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "Fz");
  Tx = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "Tx");
  Ty = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "Ty");
  Tz = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "Tz");
  dx = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "dx");
  dy = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "dy");
  dz = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "dz");
  rx = emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "rx");
  ry = emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "ry");
  rz = emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "rz");
  /* Invoke the target function */
  StaticForceTransform_FtsToTcp(Fx, Fy, Fz, Tx, Ty, Tz, dx, dy, dz, rx, ry, rz,
                                *FTtcp_sensor);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*FTtcp_sensor);
}

/*
 * File trailer for _coder_StaticForceTransform_FtsToTcp_api.c
 *
 * [EOF]
 */
