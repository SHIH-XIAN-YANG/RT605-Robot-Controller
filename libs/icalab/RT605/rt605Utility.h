/*
*  - Build    : 2022/02/17, B.R.Tseng
*  - Last Edit: 2022/03/09, B.R.Tseng
*/
#pragma once
#include<array>
#include"C:/lib/global_source/BRT_Math.h"
#include"d:/program/sln/rtx_test/rtss/robot_parameter.h"

inline void JointToCount(double* q, double* pulse) {
	pulse[0] = ((q[0] - M_PI_2) * (80 / PPU)) + Zeros1;
	pulse[1] = ((q[1] - M_PI_2) * (100 / PPU)) + Zeros2;
	pulse[2] = (q[2] * (-80 / PPU))  + Zeros3;
	pulse[3] = (q[3] * (-81 / PPU))  + Zeros4;
	pulse[4] = (q[4] * (-80 / PPU))  + Zeros5;
	pulse[5] = ((q[5] * -50) - q[4]) / PPU  + Zeros6;
}
//
inline void CountToJoint(double* pulse, double* q) {
	q[0] = M_PI_2 + (pulse[0]  - Zeros1) / 80 * PPU;
	q[1] = M_PI_2 + (pulse[1]  - Zeros2) / 100 * PPU;
	q[2] = (pulse[2] - Zeros3) / -80 * PPU;
	q[3] = (pulse[3] - Zeros4) / -81 * PPU;
	q[4] = (pulse[4] - Zeros5) / -80 * PPU;
	q[5] = ((pulse[5] - Zeros6) * PPU + q[4]) / -50;
}
//
inline void JointToMotor(double* q, double* m) {
	m[0] = q[0] * 80.0;
	m[1] = q[1] * 100.0;
	m[2] = q[2] * -80.0;
	m[3] = q[3] * -81.0;
	m[4] = q[4] * -80;
	m[5] = (q[5] * -50) - q[4];
}
inline void Joint6ToMotor6(double* q, double* m) {
	m[5] = (q[5] * -50) - q[4];
}
//
inline void MotorToJoint(double* m, double* q) {
	q[0] = m[0] / 80.0;
	q[1] = m[1] / 100.0;
	q[2] = m[2] / -80.0;
	q[3] = m[3] / -81.0;
	q[4] = m[4] / -80.0;
	q[5] = (m[5] + q[4]) / -50.0;
}
//
inline void JointToCount(std::array<double, 6>& target) {
	target[0] = ((target[0] - M_PI_2) * (80 / PPU)) + Zeros1;
	target[1] = ((target[1] - M_PI_2) * (100 / PPU)) + Zeros2;
	target[2] = (target[2] * (-80 / PPU)) + Zeros3;
	target[3] = (target[3] * (-81 / PPU)) + Zeros4;
	target[5] = ((target[5] * -50) - target[4]) / PPU + Zeros6;
	target[4] = (target[4] * (-80 / PPU)) + Zeros5;
}
//
inline void CountToJoint(std::array<double, 6>& target) {
	target[0] = M_PI_2 + (target[0] - Zeros1) / 80 * PPU;
	target[1] = M_PI_2 + (target[1] - Zeros2) / 100 * PPU;
	target[2] = (target[2] - Zeros3) / -80 * PPU;
	target[3] = (target[3] - Zeros4) / -81 * PPU;
	target[4] = (target[4] - Zeros5) / -80 * PPU;
	target[5] = ((target[5] - Zeros6) * PPU + target[4]) / -50;
}
//
inline void CountToMotor(std::array<double, 6>& target) {
	target[0] = M_PI_2*80.0 + (target[0] - Zeros1)* PPU;
	target[1] = M_PI_2*100.0 + (target[1] - Zeros2)  * PPU;
	target[2] = (target[2] - Zeros3) * -PPU;
	target[3] = (target[3] - Zeros4) * -PPU;
	target[4] = (target[4] - Zeros5) * -PPU;
	target[5] = ((target[5] - Zeros6) * -PPU - target[4]) ;
}
//
inline void JointToMotor(std::array<double, 6>& target) {
	target[0] = target[0] * 80.0;
	target[1] = target[1] * 100.0;
	target[2] = target[2] * -80.0;
	target[3] = target[3] * -81.0;
	target[5] = (target[5] * -50) - target[4];
	target[4] = target[4] * -80;
	
}
inline void Joint6ToMotor6(std::array<double, 6>& target) {
	target[5] = (target[5] * -50) - target[4];
}
//
inline void MotorToJoint(std::array<double, 6>& target) {
	target[0] = target[0] / 80.0;
	target[1] = target[1] / 100.0;
	target[2] = target[2] / 80.0;
	target[3] = target[3] / 81.0;
	target[4] = target[4] / 80.0;
	target[5] = (target[5] + target[4]) / 50.0;
}