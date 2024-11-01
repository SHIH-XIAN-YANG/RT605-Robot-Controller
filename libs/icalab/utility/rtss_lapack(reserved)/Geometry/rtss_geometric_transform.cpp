/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2022/08/25, b.r.tseng
*/
#include"rtss_geometric_transform.h"

void StaticForceTransform_FtsToTcp(double* transform_fts_tcp,
								   double* FTS,
                                   double* Ftcp)
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
/*  */
  t2 = cos(transform_fts_tcp[3]);
  t3 = cos(transform_fts_tcp[4]);
  t4 = cos(transform_fts_tcp[5]);
  t5 = sin(transform_fts_tcp[3]);
  t6 = sin(transform_fts_tcp[4]);
  t7 = sin(transform_fts_tcp[5]);
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
  t66_tmp = transform_fts_tcp[2] * t6;
  t66 = ((transform_fts_tcp[0] * t3 * t4 + transform_fts_tcp[1] * t3 * t7) + -(t66_tmp * t10)) + -(t66_tmp * t13);
  t68 = 1.0 / (((t32 + t33) + t34) + t35);
  t69_tmp = transform_fts_tcp[2] * t2 * t3;
  b_t69_tmp = transform_fts_tcp[1] * t9;
  c_t69_tmp = transform_fts_tcp[1] * t12;
  t66_tmp = transform_fts_tcp[0] * t9;
  t70 = transform_fts_tcp[0] * t12;
  t17 = ((((((transform_fts_tcp[0] * t23 + transform_fts_tcp[1] * t20) + t69_tmp * t10) + t69_tmp * t13) +
           t66_tmp * t17) +
          t70 * t17) +
         -(b_t69_tmp * t16)) +
        -(c_t69_tmp * t16);
  t16 = transform_fts_tcp[2] * t3 * t5;
  t70 = ((((((transform_fts_tcp[0] * t21 + transform_fts_tcp[1] * t22) + b_t69_tmp * t14) + c_t69_tmp * t14) +
           t16 * t10) +
          t16 * t13) +
         -(t66_tmp * t15)) +
        -(t70 * t15);
  t9 = 1.0 / (((((((t8 * t33 + t8 * t34) + t11 * t32) + t8 * t35) + t11 * t33) +
                t11 * t34) +
               t11 * t35) +
              t8 * t32);
  Ftcp[0] = (-FTS[2] * t6 + FTS[0] * t3 * t4) + FTS[1] * t3 * t7;
  Ftcp[1] = (-FTS[0] * t64 + FTS[1] * t61) + FTS[2] * t3 * t5;
  Ftcp[2] = (FTS[0] * t62 - FTS[1] * t65) + FTS[2] * t2 * t3;
  t12 = t2 * t3;
  t69_tmp = t3 * t5;
  Ftcp[3] = ((((-FTS[5] * t6 - FTS[2] * (t12 * t70 * t9 - t69_tmp * t17 * t9)) -
                       FTS[0] * (t62 * t70 * t9 + t64 * t17 * t9)) +
                      FTS[1] * (t61 * t17 * t9 + t65 * t70 * t9)) +
                     FTS[3] * t3 * t4) +
                    FTS[4] * t3 * t7;
  t16 = t3 * t4;
  t66_tmp = t3 * t7;
  Ftcp[4] =
      ((((-FTS[3] * t64 + FTS[4] * t61) + FTS[0] * (t62 * t66 * t68 - t16 * t17 * t9)) -
        FTS[1] * (t65 * t66 * t68 + t66_tmp * t17 * t9)) +
       FTS[2] * (t6 * t17 * t9 + t12 * t66 * t68)) +
      FTS[5] * t3 * t5;
  Ftcp[5] =
      ((((FTS[3] * t62 - FTS[4] * t65) + FTS[0] * (t64 * t66 * t68 + t16 * t70 * t9)) -
        FTS[1] * (t61 * t66 * t68 - t66_tmp * t70 * t9)) -
       FTS[2] * (t6 * t70 * t9 + t69_tmp * t66 * t68)) +
      FTS[5] * t2 * t3;
}
//
void StaticForceTransform_FtsToTcp(std::array<double, 6>& transform_fts_tcp,
								   std::array<double, 6>& FTS,
                                   std::array<double, 6>& Ftcp)
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
/*  */
  t2 = cos(transform_fts_tcp[3]);
  t3 = cos(transform_fts_tcp[4]);
  t4 = cos(transform_fts_tcp[5]);
  t5 = sin(transform_fts_tcp[3]);
  t6 = sin(transform_fts_tcp[4]);
  t7 = sin(transform_fts_tcp[5]);
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
  t66_tmp = transform_fts_tcp[2] * t6;
  t66 = ((transform_fts_tcp[0] * t3 * t4 + transform_fts_tcp[1] * t3 * t7) + -(t66_tmp * t10)) + -(t66_tmp * t13);
  t68 = 1.0 / (((t32 + t33) + t34) + t35);
  t69_tmp = transform_fts_tcp[2] * t2 * t3;
  b_t69_tmp = transform_fts_tcp[1] * t9;
  c_t69_tmp = transform_fts_tcp[1] * t12;
  t66_tmp = transform_fts_tcp[0] * t9;
  t70 = transform_fts_tcp[0] * t12;
  t17 = ((((((transform_fts_tcp[0] * t23 + transform_fts_tcp[1] * t20) + t69_tmp * t10) + t69_tmp * t13) +
           t66_tmp * t17) +
          t70 * t17) +
         -(b_t69_tmp * t16)) +
        -(c_t69_tmp * t16);
  t16 = transform_fts_tcp[2] * t3 * t5;
  t70 = ((((((transform_fts_tcp[0] * t21 + transform_fts_tcp[1] * t22) + b_t69_tmp * t14) + c_t69_tmp * t14) +
           t16 * t10) +
          t16 * t13) +
         -(t66_tmp * t15)) +
        -(t70 * t15);
  t9 = 1.0 / (((((((t8 * t33 + t8 * t34) + t11 * t32) + t8 * t35) + t11 * t33) +
                t11 * t34) +
               t11 * t35) +
              t8 * t32);
  Ftcp[0] = (-FTS[2] * t6 + FTS[0] * t3 * t4) + FTS[1] * t3 * t7;
  Ftcp[1] = (-FTS[0] * t64 + FTS[1] * t61) + FTS[2] * t3 * t5;
  Ftcp[2] = (FTS[0] * t62 - FTS[1] * t65) + FTS[2] * t2 * t3;
  t12 = t2 * t3;
  t69_tmp = t3 * t5;
  Ftcp[3] = ((((-FTS[5] * t6 - FTS[2] * (t12 * t70 * t9 - t69_tmp * t17 * t9)) -
                       FTS[0] * (t62 * t70 * t9 + t64 * t17 * t9)) +
                      FTS[1] * (t61 * t17 * t9 + t65 * t70 * t9)) +
                     FTS[3] * t3 * t4) +
                    FTS[4] * t3 * t7;
  t16 = t3 * t4;
  t66_tmp = t3 * t7;
  Ftcp[4] =
      ((((-FTS[3] * t64 + FTS[4] * t61) + FTS[0] * (t62 * t66 * t68 - t16 * t17 * t9)) -
        FTS[1] * (t65 * t66 * t68 + t66_tmp * t17 * t9)) +
       FTS[2] * (t6 * t17 * t9 + t12 * t66 * t68)) +
      FTS[5] * t3 * t5;
  Ftcp[5] =
      ((((FTS[3] * t62 - FTS[4] * t65) + FTS[0] * (t64 * t66 * t68 + t16 * t70 * t9)) -
        FTS[1] * (t61 * t66 * t68 - t66_tmp * t70 * t9)) -
       FTS[2] * (t6 * t70 * t9 + t69_tmp * t66 * t68)) +
      FTS[5] * t2 * t3;
}
//
void StaticForceTransform_FtsToTcp(std::array<double, 6>& transform_fts_tcp,
								   double* FTS,
                                   double* Ftcp)
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
/*  */
  t2 = cos(transform_fts_tcp[3]);
  t3 = cos(transform_fts_tcp[4]);
  t4 = cos(transform_fts_tcp[5]);
  t5 = sin(transform_fts_tcp[3]);
  t6 = sin(transform_fts_tcp[4]);
  t7 = sin(transform_fts_tcp[5]);
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
  t66_tmp = transform_fts_tcp[2] * t6;
  t66 = ((transform_fts_tcp[0] * t3 * t4 + transform_fts_tcp[1] * t3 * t7) + -(t66_tmp * t10)) + -(t66_tmp * t13);
  t68 = 1.0 / (((t32 + t33) + t34) + t35);
  t69_tmp = transform_fts_tcp[2] * t2 * t3;
  b_t69_tmp = transform_fts_tcp[1] * t9;
  c_t69_tmp = transform_fts_tcp[1] * t12;
  t66_tmp = transform_fts_tcp[0] * t9;
  t70 = transform_fts_tcp[0] * t12;
  t17 = ((((((transform_fts_tcp[0] * t23 + transform_fts_tcp[1] * t20) + t69_tmp * t10) + t69_tmp * t13) +
           t66_tmp * t17) +
          t70 * t17) +
         -(b_t69_tmp * t16)) +
        -(c_t69_tmp * t16);
  t16 = transform_fts_tcp[2] * t3 * t5;
  t70 = ((((((transform_fts_tcp[0] * t21 + transform_fts_tcp[1] * t22) + b_t69_tmp * t14) + c_t69_tmp * t14) +
           t16 * t10) +
          t16 * t13) +
         -(t66_tmp * t15)) +
        -(t70 * t15);
  t9 = 1.0 / (((((((t8 * t33 + t8 * t34) + t11 * t32) + t8 * t35) + t11 * t33) +
                t11 * t34) +
               t11 * t35) +
              t8 * t32);
  Ftcp[0] = (-FTS[2] * t6 + FTS[0] * t3 * t4) + FTS[1] * t3 * t7;
  Ftcp[1] = (-FTS[0] * t64 + FTS[1] * t61) + FTS[2] * t3 * t5;
  Ftcp[2] = (FTS[0] * t62 - FTS[1] * t65) + FTS[2] * t2 * t3;
  t12 = t2 * t3;
  t69_tmp = t3 * t5;
  Ftcp[3] = ((((-FTS[5] * t6 - FTS[2] * (t12 * t70 * t9 - t69_tmp * t17 * t9)) -
                       FTS[0] * (t62 * t70 * t9 + t64 * t17 * t9)) +
                      FTS[1] * (t61 * t17 * t9 + t65 * t70 * t9)) +
                     FTS[3] * t3 * t4) +
                    FTS[4] * t3 * t7;
  t16 = t3 * t4;
  t66_tmp = t3 * t7;
  Ftcp[4] =
      ((((-FTS[3] * t64 + FTS[4] * t61) + FTS[0] * (t62 * t66 * t68 - t16 * t17 * t9)) -
        FTS[1] * (t65 * t66 * t68 + t66_tmp * t17 * t9)) +
       FTS[2] * (t6 * t17 * t9 + t12 * t66 * t68)) +
      FTS[5] * t3 * t5;
  Ftcp[5] =
      ((((FTS[3] * t62 - FTS[4] * t65) + FTS[0] * (t64 * t66 * t68 + t16 * t70 * t9)) -
        FTS[1] * (t61 * t66 * t68 - t66_tmp * t70 * t9)) -
       FTS[2] * (t6 * t70 * t9 + t69_tmp * t66 * t68)) +
      FTS[5] * t2 * t3;
}