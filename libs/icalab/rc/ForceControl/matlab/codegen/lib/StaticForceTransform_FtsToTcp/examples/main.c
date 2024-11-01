/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "StaticForceTransform_FtsToTcp.h"
#include "StaticForceTransform_FtsToTcp_terminate.h"

/* Function Declarations */
static double argInit_real_T(void);

static void c_main_StaticForceTransform_Fts(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void c_main_StaticForceTransform_Fts(void)
{
  double FTtcp_sensor[6];
  double Fx_tmp;
  /* Initialize function 'StaticForceTransform_FtsToTcp' input arguments. */
  Fx_tmp = argInit_real_T();
  /* Call the entry-point 'StaticForceTransform_FtsToTcp'. */
  StaticForceTransform_FtsToTcp(Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp,
                                Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp, Fx_tmp,
                                FTtcp_sensor);
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  c_main_StaticForceTransform_Fts();
  /* Terminate the application.
You do not need to do this more than one time. */
  StaticForceTransform_FtsToTcp_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
