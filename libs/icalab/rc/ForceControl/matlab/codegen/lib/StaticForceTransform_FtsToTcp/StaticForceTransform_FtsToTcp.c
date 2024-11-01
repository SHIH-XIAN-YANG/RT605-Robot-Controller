/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: StaticForceTransform_FtsToTcp.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 26-Aug-2022 17:13:51
 */

/* Include Files */
#include "StaticForceTransform_FtsToTcp.h"
#include <math.h>

/* Function Definitions */
/*
 * STATICFORCETRANSFORM_FTSTOTCP
 *     FTTCP_SENSOR =
 * STATICFORCETRANSFORM_FTSTOTCP(FX,FY,FZ,TX,TY,TZ,DX,DY,DZ,RX,RY,RZ)
 *
 * Arguments    : double Fx
 *                double Fy
 *                double Fz
 *                double Tx
 *                double Ty
 *                double Tz
 *                double dx
 *                double dy
 *                double dz
 *                double rx
 *                double ry
 *                double rz
 *                double FTtcp_sensor[6]
 * Return Type  : void
 */
void StaticForceTransform_FtsToTcp(double Fx, double Fy, double Fz, double Tx,
                                   double Ty, double Tz, double dx, double dy,
                                   double dz, double rx, double ry, double rz,
                                   double FTtcp_sensor[6])
{
  double b_t69_tmp;
  double c_t69_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t2;
  double t20;
  double t21;
  double t22;
  double t23;
  double t3;
  double t32;
  double t33;
  double t34;
  double t35;
  double t4;
  double t5;
  double t6;
  double t61;
  double t62;
  double t64;
  double t65;
  double t66;
  double t66_tmp;
  double t68;
  double t69_tmp;
  double t7;
  double t70;
  double t8;
  double t9;

  t2 = cos(rx);
  t3 = cos(ry);
  t4 = cos(rz);
  t5 = sin(rx);
  t6 = sin(ry);
  t7 = sin(rz);
  t8 = t2 * t2;
  t9 = t3 * t3;
  t10 = t4 * t4;
  t11 = t5 * t5;
  t12 = t6 * t6;
  t13 = t7 * t7;
  t14 = t2 * t4;
  t15 = t2 * t7;
  t16 = t4 * t5;
  t17 = t5 * t7;
  t20 = t6 * t15;
  t21 = t6 * t16;
  t22 = t6 * t17;
  t23 = t6 * t14;
  t32 = t9 * t10;
  t33 = t9 * t13;
  t34 = t10 * t12;
  t35 = t12 * t13;
  t61 = t14 + t22;
  t62 = t17 + t23;
  t64 = t15 + -t21;
  t65 = t16 + -t20;
  t66_tmp = dz * t6;
  t66 = ((dx * t3 * t4 + dy * t3 * t7) + -(t66_tmp * t10)) + -(t66_tmp * t13);
  t68 = 1.0 / (((t32 + t33) + t34) + t35);
  t69_tmp = dz * t2 * t3;
  b_t69_tmp = dy * t9;
  c_t69_tmp = dy * t12;
  t66_tmp = dx * t9;
  t70 = dx * t12;
  t17 = ((((((dx * t23 + dy * t20) + t69_tmp * t10) + t69_tmp * t13) +
           t66_tmp * t17) +
          t70 * t17) +
         -(b_t69_tmp * t16)) +
        -(c_t69_tmp * t16);
  t16 = dz * t3 * t5;
  t70 = ((((((dx * t21 + dy * t22) + b_t69_tmp * t14) + c_t69_tmp * t14) +
           t16 * t10) +
          t16 * t13) +
         -(t66_tmp * t15)) +
        -(t70 * t15);
  t9 = 1.0 / (((((((t8 * t33 + t8 * t34) + t11 * t32) + t8 * t35) + t11 * t33) +
                t11 * t34) +
               t11 * t35) +
              t8 * t32);
  FTtcp_sensor[0] = (-Fz * t6 + Fx * t3 * t4) + Fy * t3 * t7;
  FTtcp_sensor[1] = (-Fx * t64 + Fy * t61) + Fz * t3 * t5;
  FTtcp_sensor[2] = (Fx * t62 - Fy * t65) + Fz * t2 * t3;
  t12 = t2 * t3;
  t69_tmp = t3 * t5;
  FTtcp_sensor[3] = ((((-Tz * t6 - Fz * (t12 * t70 * t9 - t69_tmp * t17 * t9)) -
                       Fx * (t62 * t70 * t9 + t64 * t17 * t9)) +
                      Fy * (t61 * t17 * t9 + t65 * t70 * t9)) +
                     Tx * t3 * t4) +
                    Ty * t3 * t7;
  t16 = t3 * t4;
  t66_tmp = t3 * t7;
  FTtcp_sensor[4] =
      ((((-Tx * t64 + Ty * t61) + Fx * (t62 * t66 * t68 - t16 * t17 * t9)) -
        Fy * (t65 * t66 * t68 + t66_tmp * t17 * t9)) +
       Fz * (t6 * t17 * t9 + t12 * t66 * t68)) +
      Tz * t3 * t5;
  FTtcp_sensor[5] =
      ((((Tx * t62 - Ty * t65) + Fx * (t64 * t66 * t68 + t16 * t70 * t9)) -
        Fy * (t61 * t66 * t68 - t66_tmp * t70 * t9)) -
       Fz * (t6 * t70 * t9 + t69_tmp * t66 * t68)) +
      Tz * t2 * t3;
}

/*
 * File trailer for StaticForceTransform_FtsToTcp.c
 *
 * [EOF]
 */
