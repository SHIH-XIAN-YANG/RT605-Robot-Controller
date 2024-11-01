#pragma once
/*
* - Create: 2021/02/16, B. R. Tseng
* - Edit: 2022/10/02, B. R. Tseng
*/
//#define _USE_MATH_DEFINES // for C++
#include<cmath>
#include<vector>
#include<string>
#include<array>

#include"eigen-3.4.0/Eigen/Dense"
#include"eigen-3.4.0/Eigen/Geometry"
#include"../libs/icalab/utility/utility_math.h"
#include"../libs/icalab/rc/ParameterEditor/ParameterEditor.h"

// ------------------- Define Singularities: ---------------------------------
#define WRIST_THRESHOLD M_PI_6 // +- 30.0 deg.
#define ELBOW_THRESHOLD 20 // +- 20.0 mm
#define SHOULDER_THRESHOLD 20 // +- 20.0 mm
//
enum class RobotError : unsigned short {
	Safe = 0,
	J1POT			 = 0x0041, // 0b0000 000001 000001
	J1NOT			 = 0x0001, // 0b0000 000000 000001
	J2POT			 = 0x0082, // 0b0000 000010 000010
	J2NOT			 = 0x0002, // 0b0000 000000 000010
	J3POT			 = 0x0104, // 0b0000 000100 000100
	J3NOT			 = 0x0004, // 0b0000 000000 000100
	J4POT			 = 0x0208, // 0b0000 001000 001000
	J4NOT			 = 0x0008, // 0b0000 000000 001000
	J5POT			 = 0x0410, // 0b0000 010000 010000
	J5NOT			 = 0x0010, // 0b0000 000000 010000
	J6POT			 = 0x0820, // 0b0000 100000 100000
	J6NOT			 = 0x0020, // 0b0000 000000 100000

	ShoulderSingular = 0x1000,//0b0001 0000 0000 0000,
	WristSingular	 = 0x2000,//0b0010 0000 0000 0000,
	ElbowSingular	 = 0x3000,//0b0011 0000 0000 0000,
};
// ----------------- Robot Multi-IK-Solution for different posture: ---------------------------
#define UNIT_RAD 0
#define UNIT_DEGREE 1
#define UNIT_PULSE 2
enum  Robot_Postures :unsigned short {
	MarkUp_ElbowUp_WristFronted     = 0, 
	MarkUp_ElbowUp_WristFlipped     = 1, 
	MarkUp_ElbowDown_WristFronted   = 2, 
	MarkUp_ElbowDown_WristFlipped   = 3, 
	MarkDown_ElbowUp_WristFronted   = 4, 
	MarkDown_ElbowUp_WristFlipped   = 5, 
	MarkDown_ElbowDown_WristFronted = 6, 
	MarkDown_ElbowDown_WristFlipped = 7
};

// -------------------------------------------------------------
enum class DH_Type : char
{
	Standard = 'S',		// Standard version
	Modified = 'M'		// Modified version
};
// Inverse Dynamics joint angle solution structure:
template<class ValType>
struct IK_Sol {
	ValType q1[2];
	ValType q2[4];
	ValType q3[4];
	ValType q4[8];
	ValType q5[8];
	ValType q6[8];
	std::array<std::array<ValType*, 6>, 8> Sol;

	IK_Sol(void) {

		class std::array<std::array<ValType*, 6>, 8>::iterator itSol = Sol.begin();
		// ----- Solution 1: -----
		itSol->at(0) = &q1[0]; // Joint 1-1
		itSol->at(1) = &q2[0]; // Joint 2-1
		itSol->at(2) = &q3[0]; // Joint 3-1
		itSol->at(3) = &q4[0]; // Joint 4-1
		itSol->at(4) = &q5[0]; // Joint 5-1
		itSol->at(5) = &q6[0]; // Joint 6-1
		// ----- Solution 2: -----
		++itSol;
		itSol->at(0) = &q1[0]; // Joint 1-2
		itSol->at(1) = &q2[0]; // Joint 2-2
		itSol->at(2) = &q3[0]; // Joint 3-2
		itSol->at(3) = &q4[1]; // Joint 4-2
		itSol->at(4) = &q5[1]; // Joint 5-2
		itSol->at(5) = &q6[1]; // Joint 6-2
		// ----- Solution 3: -----
		++itSol;
		itSol->at(0) = &q1[0]; // Joint 1-3
		itSol->at(1) = &q2[1]; // Joint 2-3
		itSol->at(2) = &q3[1]; // Joint 3-3
		itSol->at(3) = &q4[2]; // Joint 4-3
		itSol->at(4) = &q5[2]; // Joint 5-3
		itSol->at(5) = &q6[2]; // Joint 6-3
		// ----- Solution 4: -----
		++itSol;
		itSol->at(0) = &q1[0]; // Joint 1-4
		itSol->at(1) = &q2[1]; // Joint 2-4
		itSol->at(2) = &q3[1]; // Joint 3-4
		itSol->at(3) = &q4[3]; // Joint 4-4
		itSol->at(4) = &q5[3]; // Joint 5-4
		itSol->at(5) = &q6[3]; // Joint 6-4
		// ----- Solution 5: -----
		++itSol;
		itSol->at(0) = &q1[1]; // Joint 1-5
		itSol->at(1) = &q2[2]; // Joint 2-5
		itSol->at(2) = &q3[2]; // Joint 3-5
		itSol->at(3) = &q4[4]; // Joint 4-5
		itSol->at(4) = &q5[4]; // Joint 5-5
		itSol->at(5) = &q6[4]; // Joint 6-5
		// ----- Solution 6: -----
		++itSol;
		itSol->at(0) = &q1[1]; // Joint 1-6
		itSol->at(1) = &q2[2]; // Joint 2-6
		itSol->at(2) = &q3[2]; // Joint 3-6
		itSol->at(3) = &q4[5]; // Joint 4-6
		itSol->at(4) = &q5[5]; // Joint 5-6
		itSol->at(5) = &q6[5]; // Joint 6-6
		// ----- Solution 7: -----
		++itSol;
		itSol->at(0) = &q1[1]; // Joint 1-7
		itSol->at(1) = &q2[3]; // Joint 2-7
		itSol->at(2) = &q3[3]; // Joint 3-7
		itSol->at(3) = &q4[6]; // Joint 4-7
		itSol->at(4) = &q5[6]; // Joint 5-7
		itSol->at(5) = &q6[6]; // Joint 6-7
		// ----- Solution 8: -----
		++itSol;
		itSol->at(0) = &q1[1]; // Joint 1-8
		itSol->at(1) = &q2[3]; // Joint 2-8
		itSol->at(2) = &q3[3]; // Joint 3-8
		itSol->at(3) = &q4[7]; // Joint 4-8
		itSol->at(4) = &q5[7]; // Joint 5-8
		itSol->at(5) = &q6[7]; // Joint 6-8
	}
	void ReadSol(unsigned short index, ValType* q) {
		for (auto& pVal : Sol.at(index))
			*(q++) = *pVal;
	}
	void WriteSol(unsigned short index, ValType* q) {

		class std::array<ValType*, 6>::iterator itJoint = Sol.at(index).begin();
		for (unsigned short i = 0; i < 6; ++i)
			**(itJoint++) = q[i];
	}
	std::array<ValType, 6>&& operator()(unsigned short index) {
		std::array<ValType, 6>&& tmp = { *(Sol.at(index).at(0)), *(Sol.at(index).at(1)), *(Sol.at(index).at(2)),
										*(Sol.at(index).at(3)), *(Sol.at(index).at(4)) , *(Sol.at(index).at(5)) };
		return std::move(tmp);
	}
	/* Example: (Write solution 1)
	*
		IK_Sol<float> q_sol1;
		float qq1[6] = { 11.0f, 21.0f, 31.0f, 41.0f, 51.0f, 61.0f };
		q_sol1.WriteSol(0, qq1);
*/
/* Example: (Read solution 1)
*
	float ans[6];
	q_sol1.ReadSol(0, ans);
	for (auto& val : ans) // print solution 1
		std::cout << val << std::endl << std::endl;
*/
/* Example: (Access or extract the solution)
*
	std::array<float, 6> qans = q_sol1(1);
	for (auto& val : qans) // print solution 2
		std::cout << val << std::endl;
*/
};
//RT605_IK_Sol8(std::array<ValType, 6>& PosVec, IK_Sol<ValType>& q_sol, bool _unit = UNIT_RAD)
// typedef void (*IK_Runtime) (std::array<double, 6>&, IK_Sol<double>&, bool);
namespace icalab {
	template<class ValType>
	class RT605{
		private:
			Eigen::Matrix<ValType, 4, 4> T67;		// from the wrist to the end-point
			Eigen::Matrix<ValType, 4, 4> T76;
			Eigen::Matrix<ValType, 4, 4> T7_tcp;	// from the end-point to the Tool-center-point (TCP)
			Eigen::Matrix<ValType, 4, 4> Ttcp_7;	// from the Tool-center-point to the end-point 
			Eigen::Matrix<ValType, 4, 4> Ttcp_6;
			Eigen::Matrix<ValType, 4, 4> T6_tcp;
			Eigen::Matrix<ValType, 4, 4> Tref_ws;  // 從 robot 座標系轉換至所設定的 工作座標系 (HRSS 中 Calibrate 功能中的 base )
			Eigen::Matrix<ValType, 4, 4> Tws_ref;  // 從 工作座標系 轉換至 robot 座標系
		public:
			KinematicParameter kinePrt;

			std::string RobotName;
			// ======================= Constructor: =======================
			RT605(void) {
				T67 = Eigen::Transform<ValType, 3, Eigen::Affine>(Eigen::Translation<ValType, 3>(Eigen::Matrix<ValType, 3, 1>(0, 0, 0.0865))).matrix();
				T76 = T67.inverse();
			}
			inline void set_TcpFrame(double* _endFlange_to_tcp) {
				memcpy(kinePrt.endFrame_to_tcp.data(), _endFlange_to_tcp, sizeof(double) * 6);
				Eigen::Affine3d&& _T7_tcp =
					Eigen::Translation3d(Eigen::Vector3d(_endFlange_to_tcp[0], _endFlange_to_tcp[1], _endFlange_to_tcp[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((_endFlange_to_tcp[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((_endFlange_to_tcp[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((_endFlange_to_tcp[3]), Eigen::Vector3d::UnitX());
				T7_tcp = _T7_tcp.matrix();
				Ttcp_7 = T7_tcp.inverse();
				Ttcp_6 = Ttcp_7 * T76;
				T6_tcp = T67 * T7_tcp;
#ifndef Windows
				RtPrintf("X: %lld (um), Y: %lld (um), Z: %lld (um), Rx: %lld (m-deg), Ry: %lld (m-deg), Rz: %lld(m-deg)\n", \
					(long long)(kinePrt.endFrame_to_tcp[0]*1000000.0), (long long)(kinePrt.endFrame_to_tcp[1] * 1000000.0), (long long)(kinePrt.endFrame_to_tcp[2] * 1000000.0), \
					(long long)(rad2deg(kinePrt.endFrame_to_tcp[3]) * 1000.0), (long long)(rad2deg(kinePrt.endFrame_to_tcp[4]) * 1000.0), (long long)(rad2deg(kinePrt.endFrame_to_tcp[5]) * 1000.0));
#endif
			}

			//
			inline void set_WorkspaceFrame(double* _ref_to_workspace) {
				memcpy(kinePrt.refFrame_to_workspace.data(), _ref_to_workspace, sizeof(double) * 6);
				Eigen::Affine3d&& _Tref_ws =
					Eigen::Translation3d(Eigen::Vector3d(_ref_to_workspace[0], _ref_to_workspace[1], _ref_to_workspace[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((_ref_to_workspace[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((_ref_to_workspace[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((_ref_to_workspace[3]), Eigen::Vector3d::UnitX());
				Tref_ws = _Tref_ws.matrix();
				Tws_ref = Tref_ws.inverse();
#ifndef Windows
				RtPrintf("X: %lld (um), Y: %lld (um), Z: %lld (um), Rx: %lld (m-deg), Ry: %lld (m-deg), Rz: %lld (m-deg)\n", \
					(long long)(kinePrt.refFrame_to_workspace[0] * 1000000.0), (long long)(kinePrt.refFrame_to_workspace[1] * 1000000.0), (long long)(kinePrt.refFrame_to_workspace[2] * 1000000.0), \
					(long long)(rad2deg(kinePrt.refFrame_to_workspace[3]) * 1000.0), (long long)(rad2deg(kinePrt.refFrame_to_workspace[4]) * 1000.0), (long long)(rad2deg(kinePrt.refFrame_to_workspace[5]) * 1000.0));
#endif
			}

			// ======================= Funtions: =======================
			// ---------------- Control Kernel: ----------------
#pragma  region Kinematics for RT605: 
			//inline void DeterminPosture(std::array<ValType, 6>& q, Robot_Postures& currentPosture) {

			//}
			inline void RT605_FK_XYZ(ValType q1, ValType q2, ValType q3, ValType q4, ValType q5, std::array<ValType, 3>& _Pos)
			{
				// Arguments:
			    //ValType q1 ~ q5
				ValType&& t2 = std::cos(q1);
				ValType&& t3 = std::cos(q2);
				ValType&& t4 = std::cos(q3);
				ValType&& t5 = std::cos(q4);
				ValType&& t6 = std::cos(q5);
				ValType&& t7 = std::sin(q1);
				ValType&& t8 = std::sin(q2);
				ValType&& t9 = std::sin(q3);
				ValType&& t10 = std::sin(q4);
				ValType&& t11 = std::sin(q5);
				ValType&& t12 = q2 + q3;
				ValType&& t13 = std::cos(t12);
				t12 = std::sin(t12);
				ValType&& ans_tmp = t2 * t3;
				ValType&& b_ans_tmp = ans_tmp * t4;
				_Pos.at(0) = ((((((((t2 * 0.03 + ans_tmp * 0.34) + b_ans_tmp / 25.0) + ans_tmp *
					t9 * 0.338) + t2 * t4 * t8 * 0.338) - t2 * t8 * t9 / 25.0) + t2
					* t6 * t12 * 0.0865) + t7 * t10 * t11 * 0.0865) + b_ans_tmp * t5 *
					t11 * 0.0865) - t2 * t5 * t8 * t9 * t11 * 0.0865;
				ans_tmp = t3 * t7;
				b_ans_tmp = t3 * t4;
				_Pos.at(1) = ((((((((t7 * 0.03 + ans_tmp * 0.34) + b_ans_tmp * t7 / 25.0) +
					ans_tmp * t9 * 0.338) + t4 * t7 * t8 * 0.338) - t2 * t10 * t11 *
					0.0865) - t7 * t8 * t9 / 25.0) + t6 * t7 * t12 * 0.0865) +
					b_ans_tmp * t5 * t7 * t11 * 0.0865) - t5 * t7 * t8 * t9 * t11 *
					0.0865;
				_Pos.at(2) = ((((t8 * 0.34 - t13 * 0.338) + t12 / 25.0) - t6 * t13 * 0.0865) + t12
					* std::sin(q4 + q5) * 0.04325) - t12 * std::sin(q4 - q5) * 0.04325;
			}
			inline void RT605_FK_XYZ(ValType* q, ValType* pos)
			{
				// Arguments:
				//ValType q1 ~ q5

				ValType q1{ (*(q)) }, q2{ (*(q + 1)) }, q3{ (*(q + 2)) }, q4{ (*(q + 3)) }, q5{ (*(q + 4)) }, q6{ (*(q + 5)) };
				ValType&& t2 = std::cos(q1);
				ValType&& t3 = std::cos(q2);
				ValType&& t4 = std::cos(q3);
				ValType&& t5 = std::cos(q4);
				ValType&& t6 = std::cos(q5);
				ValType&& t7 = std::sin(q1);
				ValType&& t8 = std::sin(q2);
				ValType&& t9 = std::sin(q3);
				ValType&& t10 = std::sin(q4);
				ValType&& t11 = std::sin(q5);
				ValType&& t12 = q2 + q3;
				ValType&& t13 = std::cos(t12);
				t12 = std::sin(t12);
				ValType&& ans_tmp = t2 * t3;
				ValType&& b_ans_tmp = ans_tmp * t4;
				*(pos) = ((((((((t2 * 0.03 + ans_tmp * 0.34) + b_ans_tmp / 25.0) + ans_tmp *
					t9 * 0.338) + t2 * t4 * t8 * 0.338) - t2 * t8 * t9 / 25.0) + t2
					* t6 * t12 * 0.0865) + t7 * t10 * t11 * 0.0865) + b_ans_tmp * t5 *
					t11 * 0.0865) - t2 * t5 * t8 * t9 * t11 * 0.0865;
				ans_tmp = t3 * t7;
				b_ans_tmp = t3 * t4;
				*(pos+1) = ((((((((t7 * 0.03 + ans_tmp * 0.34) + b_ans_tmp * t7 / 25.0) +
					ans_tmp * t9 * 0.338) + t4 * t7 * t8 * 0.338) - t2 * t10 * t11 *
					0.0865) - t7 * t8 * t9 / 25.0) + t6 * t7 * t12 * 0.0865) +
					b_ans_tmp * t5 * t7 * t11 * 0.0865) - t5 * t7 * t8 * t9 * t11 *
					0.0865;
				*(pos+2) = ((((t8 * 0.34 - t13 * 0.338) + t12 / 25.0) - t6 * t13 * 0.0865) + t12
					* std::sin(q4 + q5) * 0.04325) - t12 * std::sin(q4 - q5) * 0.04325;
			}
			//
			inline void RT605_FK_Orientation(ValType* q, ValType* Angle)
			{
				ValType q1{ (*(q)) }, q2{ (*(q + 1)) }, q3{ (*(q + 2)) }, q4{ (*(q + 3)) }, q5{ (*(q + 4)) }, q6{ (*(q + 5)) };
				ValType&& t2 = std::cos(q1);
				ValType&& t3 = std::cos(q2);
				ValType&& t4 = std::cos(q3);
				ValType&& t5 = std::cos(q4);
				ValType&& t6 = std::cos(q5);
				ValType&& t7 = std::cos(q6);
				ValType&& t8 = std::sin(q1);
				ValType&& t9 = std::sin(q2);
				ValType&& t10 = std::sin(q3);
				ValType&& t11 = std::sin(q4);
				ValType&& t12 = std::sin(q5);
				ValType&& t13 = std::sin(q6);
				ValType&& t14 = t3 * t4;
				ValType&& t15 = t3 * t10;
				ValType&& t16 = t4 * t9;
				t3 = t9 * t10;
				t9 = t15 + t16;
				t10 = t14 + -t3;
				ValType&& t29 = t2 * t14 + t2 * -t3;
				t3 = t8 * t3 + -(t8 * t14);
				t4 = t5 * t6 * t9 + t12 * t10;
				ValType && m11 = -t7 * (t12 * (t2 * t15 + t2 * t16) - t6 * (t8 * t11 + t5 * t29)) +
					t13 * (t5 * t8 - t11 * t29);
				ValType && m21 = -t7 * (t6 * (t2 * t11 + t5 * t3) + t12 * (t8 * t15 + t8 * t16)) - t13
					* (t2 * t5 - t11 * t3);
				ValType && m31 = t7 * t4 - t11 * t13 * t9;
				ValType && m32 = -t13 * t4 - t7 * t11 * t9;
				ValType && m33 = -t6 * t10 + t5 * t12 * t9;
				ValType&& m34 = sqrt(static_cast<ValType>(1.0) - m31*m31);
				//if (m31 == -0.0) m11 = 0.0;
				//if (m31 == -0.0) m21 = 0.0;
				//if (m31 == -0.0) m31 = 0.0;
				//if (m31 == -0.0) m32 = 0.0;
				//if (m31 == -0.0) m33 = 0.0;
				//if (m31 == -0.0) m34 = 0.0;
				*Angle = atan2(m32, m33);
				*(Angle + 1) = atan2(-m31, m34);
				*(Angle + 2) = atan2(-m31, -m34);
				*(Angle + 3) = atan2(m21, m11);
			}
			//
			inline void ComputeRT605_FK_Ref_Tcp(ValType* transform_ref_ws, ValType* transform_6_tcp, ValType* q, ValType* ref_tcp)
				{
					double t10;
					double t105;
					double t106;
					double t108;
					double t11;
					double t111;
					double t113;
					double t114;
					double t115;
					double t117;
					double t12;
					double t13;
					double t132;
					double t134;
					double t139;
					double t14;
					double t140;
					double t144;
					double t145;
					double t15;
					double t159;
					double t16;
					double t160;
					double t161;
					double t17;
					double t18;
					double t181;
					double t19;
					double t191;
					double t194;
					double t2;
					double t20;
					double t21;
					double t22;
					double t23;
					double t24;
					double t25;
					double t26;
					double t27;
					double t28;
					double t29;
					double t3;
					double t30;
					double t31;
					double t33;
					double t35;
					double t36;
					double t39;
					double t4;
					double t40;
					double t41;
					double t42;
					double t43;
					double t45;
					double t46;
					double t47;
					double t48;
					double t49;
					double t5;
					double t51;
					double t52;
					double t6;
					double t60;
					double t61;
					double t62;
					double t63;
					double t64;
					double t65;
					double t66;
					double t67;
					double t68;
					double t69;
					double t7;
					double t73;
					double t75;
					double t8;
					double t82;
					double t89;
					double t9;
					/*     This function was generated by the Symbolic Math Toolbox version 8.7.
					 */
					 /*     02-Oct-2022 18:36:06 */
					t2 = cos(q[0]);
					t3 = cos(q[1]);
					t4 = cos(q[2]);
					t5 = cos(q[3]);
					t6 = cos(q[4]);
					t7 = cos(q[5]);
					t8 = cos(transform_ref_ws[3]);
					t9 = cos(transform_ref_ws[4]);
					t10 = cos(transform_ref_ws[5]);
					t11 = cos(transform_6_tcp[3]);
					t12 = cos(transform_6_tcp[4]);
					t13 = cos(transform_6_tcp[5]);
					t14 = sin(q[0]);
					t15 = sin(q[1]);
					t16 = sin(q[2]);
					t17 = sin(q[3]);
					t18 = sin(q[4]);
					t19 = sin(q[5]);
					t20 = sin(transform_ref_ws[3]);
					t21 = sin(transform_ref_ws[4]);
					t22 = sin(transform_ref_ws[5]);
					t23 = sin(transform_6_tcp[3]);
					t24 = sin(transform_6_tcp[4]);
					t25 = sin(transform_6_tcp[5]);
					t26 = t8 * t8;
					t27 = t9 * t9;
					t28 = t10 * t10;
					t29 = t20 * t20;
					t30 = t21 * t21;
					t31 = t22 * t22;
					t33 = t3 * t4;
					t35 = t3 * t16;
					t36 = t4 * t15;
					t39 = t15 * t16;
					t40 = t8 * t21 * t22;
					t16 = t10 * t20;
					t41 = t16 * t21;
					t43 = t20 * t21 * t22;
					t4 = t8 * t10;
					t49 = t4 * t21;
					t42 = t14 * t39;
					t45 = t2 * t33;
					t46 = t2 * t35;
					t47 = t2 * t36;
					t48 = t14 * t33;
					t51 = t14 * t35;
					t52 = t14 * t36;
					t60 = t4 * t27;
					t61 = t4 * t30;
					t4 = t8 * t22;
					t62 = t4 * t27;
					t63 = t16 * t27;
					t64 = t4 * t30;
					t65 = t16 * t30;
					t4 = t20 * t22;
					t66 = t4 * t27;
					t67 = t4 * t30;
					t68 = t26 * t27;
					t69 = t27 * t28;
					t16 = t26 * t30;
					t26 = t27 * t29;
					t73 = t28 * t30;
					t75 = t29 * t30;
					t82 = t35 + t36;
					t89 = 1.0 / (t27 + t30);
					t4 = t33 + -t39;
					t105 = t46 + t47;
					t106 = t51 + t52;
					t108 = t5 * t18 * t82;
					t132 = (t43 + t60) + t61;
					t134 = (t49 + t66) + t67;
					t139 = (-t40 + t63) + t65;
					t140 = (-t41 + t62) + t64;
					t111 = t6 * t4;
					t113 = t45 + t2 * -t39;
					t114 = t42 + -t48;
					t115 = t6 * t105;
					t117 = t6 * t106;
					t159 = 1.0 / (((t68 + t16) + t26) + t75);
					t160 = 1.0 / (((t69 + t27 * t31) + t73) + t30 * t31);
					t27 = t5 * t6 * t82 + t18 * t4;
					t194 = 1.0 /
						(((((((t31 * t68 + t28 * t16) + t29 * t69) + t31 * t16) + t31 * t26) +
							t29 * t73) +
							t31 * t75) +
							t28 * t68);
					t4 = t2 * t17 + t5 * t114;
					t26 = t14 * t17 + t5 * t113;
					t73 = t2 * t5 + -(t17 * t114);
					t16 = t5 * t14 + -(t17 * t113);
					t144 = t18 * t26;
					t145 = t18 * t4;
					t75 = t7 * t17 * t82 + t19 * t27;
					t191 = (((((t15 * 0.34 + t35 / 25.0) + t36 / 25.0) + t39 * 0.338) +
						-(t33 * 0.338)) +
						t108 * 0.0865) +
						-(t111 * 0.0865);
					t161 = t115 + t144;
					t35 = t18 * t106 + t6 * t4;
					t4 = t18 * t105 - t6 * t26;
					t181 = t19 * t16 + -t7 * t4;
					t16 = t7 * t16 + t19 * t4;
					t46 = ((((((t2 * 0.03 + t2 * t3 * 0.34) + t45 / 25.0) + -(t2 * t39 / 25.0)) +
						t46 * 0.338) +
						t47 * 0.338) +
						t115 * 0.0865) +
						t144 * 0.0865;
					t39 = t19 * t73 + t7 * t35;
					t45 = t9 * t10 * t160;
					t18 = ((((((t14 * 0.03 + t3 * t14 * 0.34) + t48 / 25.0) + -(t42 / 25.0)) +
						t51 * 0.338) +
						t52 * 0.338) +
						t117 * 0.0865) +
						-(t145 * 0.0865);
					t6 = t108 - t111;
					t36 = t117 - t145;
					t105 = t9 * t22 * t160;
					t115 = (-t21 * t89 * t6 + t45 * t161) + t105 * t36;
					t15 = t8 * t9 * t159;
					t33 = (t15 * t6 + t134 * t161 * t194) + -t139 * t194 * t36;
					t4 = t7 * t73 + -(t19 * t35);
					t106 = t24 * t33;
					t113 = t17 * t19 * t82 - t7 * t27;
					t5 = (-t8 * t9 * t159 * t113 + t134 * t181 * t194) + t139 * t39 * t194;
					t27 = t9 * t20 * t159;
					t30 = (t27 * t113 + t132 * t39 * t194) + t140 * t181 * t194;
					t35 = (t27 * t75 + t132 * t4 * t194) + t140 * t16 * t194;
					t145 = (-(t15 * t75) + t134 * t16 * t194) + t139 * t4 * t194;
					t29 = t12 * t13;
					t26 = t29 * t5;
					t114 = t21 * t89;
					t69 = (t114 * t75 + t45 * t16) + -(t105 * t4);
					t144 = t12 * t25;
					t16 = t144 * t145;
					t4 = (-t106 + t26) + t16;
					t75 = sqrt(-(t4 * t4) + 1.0);
					t68 = (t106 * 0.0 + -(t26 * 0.0)) + -(t16 * 0.0);
					t73 = (t106 + -t26) + -t16;
					t106 = transform_ref_ws[2] * t21;
					t26 = (-(t105 * t39) + t45 * t181) + t114 * t113;
					ref_tcp[0] = (((((-t160 * (((transform_ref_ws[0] * t9 * t10 +
						transform_ref_ws[1] * t9 * t22) -
						t106 * t28) -
						t106 * t31) +
						t115 * transform_6_tcp[2]) +
						t69 * transform_6_tcp[1]) +
						transform_6_tcp[0] * t26) -
						t114 * t191) +
						t45 * t46) +
						t105 * t18;
					t106 = transform_ref_ws[2] * t9 * t20;
					t16 = t132 * t194;
					t4 = (-(t140 * t161 * t194) + t16 * t36) + t27 * t6;
					ref_tcp[1] =
						(((((-t194 *
							(((((((transform_ref_ws[0] * t41 + transform_ref_ws[1] * t43) +
								transform_ref_ws[1] * t60) -
								transform_ref_ws[0] * t62) +
								transform_ref_ws[1] * t61) -
								transform_ref_ws[0] * t64) +
								t106 * t28) +
								t106 * t31) -
							t30 * transform_6_tcp[0]) -
							t35 * transform_6_tcp[1]) +
							transform_6_tcp[2] * t4) +
							t16 * t18) -
							t140 * t194 * t46) +
						t27 * t191;
					t106 = transform_ref_ws[2] * t8 * t9;
					ref_tcp[2] =
						(((((-t194 *
							(((((((transform_ref_ws[1] * t40 + transform_ref_ws[0] * t49) -
								transform_ref_ws[1] * t63) +
								transform_ref_ws[0] * t66) -
								transform_ref_ws[1] * t65) +
								transform_ref_ws[0] * t67) +
								t106 * t28) +
								t106 * t31) +
							t33 * transform_6_tcp[2]) +
							t5 * transform_6_tcp[0]) +
							t145 * transform_6_tcp[1]) +
							t134 * t194 * t46) -
							t139 * t194 * t18) +
						t15 * t191;
					t106 = t11 * t13;
					t16 = t13 * t23;
					ref_tcp[3] = atan2(
						(-t5 * (t11 * t25 - t16 * t24) + t145 * (t106 + t23 * t24 * t25)) +
						t12 * t23 * t33,
						(t5 * (t23 * t25 + t106 * t24) - t145 * (t16 - t11 * t24 * t25)) +
						t11 * t12 * t33);
					ref_tcp[4] = atan2(t73, t68 + t75);
					ref_tcp[5] = atan2(t73, t68 - t75);
					ref_tcp[6] = atan2((-t24 * t4 - t29 * t30) - t144 * t35,
						(-t24 * t115 + t144 * t69) + t29 * t26);
				} 
			//
			inline void RT605_FK_Tcp(ValType* q, ValType* ref_tcp)
			{
				double t10;
				double t105;
				double t106;
				double t108;
				double t11;
				double t111;
				double t113;
				double t114;
				double t115;
				double t117;
				double t12;
				double t13;
				double t132;
				double t134;
				double t139;
				double t14;
				double t140;
				double t144;
				double t145;
				double t15;
				double t159;
				double t16;
				double t160;
				double t161;
				double t17;
				double t18;
				double t181;
				double t19;
				double t191;
				double t194;
				double t20;
				double t21;
				double t22;
				double t23;
				double t24;
				double t25;
				double t26;
				double t27;
				double t28;
				double t29;
				double t3;
				double t30;
				double t31;
				double t33;
				double t35;
				double t36;
				double t39;
				double t4;
				double t40;
				double t41;
				double t42;
				double t43;
				double t45;
				double t46;
				double t47;
				double t48;
				double t49;
				double t5;
				double t51;
				double t52;
				double t6;
				double t60;
				double t61;
				double t62;
				double t63;
				double t64;
				double t65;
				double t66;
				double t67;
				double t68;
				double t69;
				double t7;
				double t73;
				double t75;
				double t8;
				double t82;
				double t89;
				double t9;
				/*     This function was generated by the Symbolic Math Toolbox version 8.7.
				 */
				 /*     02-Oct-2022 18:36:06 */
				double&& t2 = cos(q[0]);
				t3 = cos(q[1]);
				t4 = cos(q[2]);
				t5 = cos(q[3]);
				t6 = cos(q[4]);
				t7 = cos(q[5]);
				t8 = cos(kinePrt.refFrame_to_workspace[3]);
				t9 = cos(kinePrt.refFrame_to_workspace[4]);
				t10 = cos(kinePrt.refFrame_to_workspace[5]);
				t11 = cos(kinePrt.endFrame_to_tcp[3]);
				t12 = cos(kinePrt.endFrame_to_tcp[4]);
				t13 = cos(kinePrt.endFrame_to_tcp[5]);
				t14 = sin(q[0]);
				t15 = sin(q[1]);
				t16 = sin(q[2]);
				t17 = sin(q[3]);
				t18 = sin(q[4]);
				t19 = sin(q[5]);
				t20 = sin(kinePrt.refFrame_to_workspace[3]);
				t21 = sin(kinePrt.refFrame_to_workspace[4]);
				t22 = sin(kinePrt.refFrame_to_workspace[5]);
				t23 = sin(kinePrt.endFrame_to_tcp[3]);
				t24 = sin(kinePrt.endFrame_to_tcp[4]);
				t25 = sin(kinePrt.endFrame_to_tcp[5]);
				t26 = t8 * t8;
				t27 = t9 * t9;
				t28 = t10 * t10;
				t29 = t20 * t20;
				t30 = t21 * t21;
				t31 = t22 * t22;
				t33 = t3 * t4;
				t35 = t3 * t16;
				t36 = t4 * t15;
				t39 = t15 * t16;
				t40 = t8 * t21 * t22;
				t16 = t10 * t20;
				t41 = t16 * t21;
				t43 = t20 * t21 * t22;
				t4 = t8 * t10;
				t49 = t4 * t21;
				t42 = t14 * t39;
				t45 = t2 * t33;
				t46 = t2 * t35;
				t47 = t2 * t36;
				t48 = t14 * t33;
				t51 = t14 * t35;
				t52 = t14 * t36;
				t60 = t4 * t27;
				t61 = t4 * t30;
				t4 = t8 * t22;
				t62 = t4 * t27;
				t63 = t16 * t27;
				t64 = t4 * t30;
				t65 = t16 * t30;
				t4 = t20 * t22;
				t66 = t4 * t27;
				t67 = t4 * t30;
				t68 = t26 * t27;
				t69 = t27 * t28;
				t16 = t26 * t30;
				t26 = t27 * t29;
				t73 = t28 * t30;
				t75 = t29 * t30;
				t82 = t35 + t36;
				t89 = 1.0 / (t27 + t30);
				t4 = t33 + -t39;
				t105 = t46 + t47;
				t106 = t51 + t52;
				t108 = t5 * t18 * t82;
				t132 = (t43 + t60) + t61;
				t134 = (t49 + t66) + t67;
				t139 = (-t40 + t63) + t65;
				t140 = (-t41 + t62) + t64;
				t111 = t6 * t4;
				t113 = t45 + t2 * -t39;
				t114 = t42 + -t48;
				t115 = t6 * t105;
				t117 = t6 * t106;
				t159 = 1.0 / (((t68 + t16) + t26) + t75);
				t160 = 1.0 / (((t69 + t27 * t31) + t73) + t30 * t31);
				t27 = t5 * t6 * t82 + t18 * t4;
				t194 = 1.0 /
					(((((((t31 * t68 + t28 * t16) + t29 * t69) + t31 * t16) + t31 * t26) +
						t29 * t73) +
						t31 * t75) +
						t28 * t68);
				t4 = t2 * t17 + t5 * t114;
				t26 = t14 * t17 + t5 * t113;
				t73 = t2 * t5 + -(t17 * t114);
				t16 = t5 * t14 + -(t17 * t113);
				t144 = t18 * t26;
				t145 = t18 * t4;
				t75 = t7 * t17 * t82 + t19 * t27;
				t191 = (((((t15 * 0.34 + t35 / 25.0) + t36 / 25.0) + t39 * 0.338) +
					-(t33 * 0.338)) +
					t108 * 0.0865) +
					-(t111 * 0.0865);
				t161 = t115 + t144;
				t35 = t18 * t106 + t6 * t4;
				t4 = t18 * t105 - t6 * t26;
				t181 = t19 * t16 + -t7 * t4;
				t16 = t7 * t16 + t19 * t4;
				t46 = ((((((t2 * 0.03 + t2 * t3 * 0.34) + t45 / 25.0) + -(t2 * t39 / 25.0)) +
					t46 * 0.338) +
					t47 * 0.338) +
					t115 * 0.0865) +
					t144 * 0.0865;
				t39 = t19 * t73 + t7 * t35;
				t45 = t9 * t10 * t160;
				t18 = ((((((t14 * 0.03 + t3 * t14 * 0.34) + t48 / 25.0) + -(t42 / 25.0)) +
					t51 * 0.338) +
					t52 * 0.338) +
					t117 * 0.0865) +
					-(t145 * 0.0865);
				t6 = t108 - t111;
				t36 = t117 - t145;
				t105 = t9 * t22 * t160;
				t115 = (-t21 * t89 * t6 + t45 * t161) + t105 * t36;
				t15 = t8 * t9 * t159;
				t33 = (t15 * t6 + t134 * t161 * t194) + -t139 * t194 * t36;
				t4 = t7 * t73 + -(t19 * t35);
				t106 = t24 * t33;
				t113 = t17 * t19 * t82 - t7 * t27;
				t5 = (-t8 * t9 * t159 * t113 + t134 * t181 * t194) + t139 * t39 * t194;
				t27 = t9 * t20 * t159;
				t30 = (t27 * t113 + t132 * t39 * t194) + t140 * t181 * t194;
				t35 = (t27 * t75 + t132 * t4 * t194) + t140 * t16 * t194;
				t145 = (-(t15 * t75) + t134 * t16 * t194) + t139 * t4 * t194;
				t29 = t12 * t13;
				t26 = t29 * t5;
				t114 = t21 * t89;
				t69 = (t114 * t75 + t45 * t16) + -(t105 * t4);
				t144 = t12 * t25;
				t16 = t144 * t145;
				t4 = (-t106 + t26) + t16;
				t75 = sqrt(-(t4 * t4) + 1.0);
				t68 = (t106 * 0.0 + -(t26 * 0.0)) + -(t16 * 0.0);
				t73 = (t106 + -t26) + -t16;
				t106 = kinePrt.refFrame_to_workspace[2] * t21;
				t26 = (-(t105 * t39) + t45 * t181) + t114 * t113;
				ref_tcp[0] = (((((-t160 * (((kinePrt.refFrame_to_workspace[0] * t9 * t10 +
					kinePrt.refFrame_to_workspace[1] * t9 * t22) -
					t106 * t28) -
					t106 * t31) +
					t115 * kinePrt.endFrame_to_tcp[2]) +
					t69 * kinePrt.endFrame_to_tcp[1]) +
					kinePrt.endFrame_to_tcp[0] * t26) -
					t114 * t191) +
					t45 * t46) +
					t105 * t18;
				t106 = kinePrt.refFrame_to_workspace[2] * t9 * t20;
				t16 = t132 * t194;
				t4 = (-(t140 * t161 * t194) + t16 * t36) + t27 * t6;
				ref_tcp[1] =
					(((((-t194 *
						(((((((kinePrt.refFrame_to_workspace[0] * t41 + kinePrt.refFrame_to_workspace[1] * t43) +
							kinePrt.refFrame_to_workspace[1] * t60) -
							kinePrt.refFrame_to_workspace[0] * t62) +
							kinePrt.refFrame_to_workspace[1] * t61) -
							kinePrt.refFrame_to_workspace[0] * t64) +
							t106 * t28) +
							t106 * t31) -
						t30 * kinePrt.endFrame_to_tcp[0]) -
						t35 * kinePrt.endFrame_to_tcp[1]) +
						kinePrt.endFrame_to_tcp[2] * t4) +
						t16 * t18) -
						t140 * t194 * t46) +
					t27 * t191;
				t106 = kinePrt.refFrame_to_workspace[2] * t8 * t9;
				ref_tcp[2] =
					(((((-t194 *
						(((((((kinePrt.refFrame_to_workspace[1] * t40 + kinePrt.refFrame_to_workspace[0] * t49) -
							kinePrt.refFrame_to_workspace[1] * t63) +
							kinePrt.refFrame_to_workspace[0] * t66) -
							kinePrt.refFrame_to_workspace[1] * t65) +
							kinePrt.refFrame_to_workspace[0] * t67) +
							t106 * t28) +
							t106 * t31) +
						t33 * kinePrt.endFrame_to_tcp[2]) +
						t5 * kinePrt.endFrame_to_tcp[0]) +
						t145 * kinePrt.endFrame_to_tcp[1]) +
						t134 * t194 * t46) -
						t139 * t194 * t18) +
					t15 * t191;
				t106 = t11 * t13;
				t16 = t13 * t23;
				ref_tcp[3] = atan2(
					(-t5 * (t11 * t25 - t16 * t24) + t145 * (t106 + t23 * t24 * t25)) +
					t12 * t23 * t33,
					(t5 * (t23 * t25 + t106 * t24) - t145 * (t16 - t11 * t24 * t25)) +
					t11 * t12 * t33);
				ref_tcp[4] = atan2(t73, t68 + t75);
				ref_tcp[5] = atan2(t73, t68 - t75);
				ref_tcp[6] = atan2((-t24 * t4 - t29 * t30) - t144 * t35,
					(-t24 * t115 + t144 * t69) + t29 * t26);
			}
			//
			inline void RT605_IK_Sol8(std::array<ValType, 6>& PosVec, IK_Sol<ValType>& q_sol, bool _unit = UNIT_RAD) {
				Eigen::Affine3d&& T0_tcp =
					Eigen::Translation3d(Eigen::Vector3d(PosVec[0], PosVec[1], PosVec[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((PosVec[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((PosVec[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((PosVec[3]), Eigen::Vector3d::UnitX());
				Eigen::Matrix<ValType, 4, 4>&& T06 = T0_tcp.matrix() * Ttcp_6;
				double x{ T06(0, 3) }, y{ T06(1, 3) }, z{ T06(2, 3) };
				//std::cout << x << ", " << y << ", " << z << std::endl;
				// --------------------------- Compute the inverse kinematics: -------------------------------------------
				// Compute solution 1:
				q_sol.q1[0] = static_cast<ValType>(atan2(y, x)); // q1-1
				q_sol.q1[1] = q_sol.q1[0] + M_PI;
				//k1 = x * cos(theta1_1) + y * sin(theta1_1);
				ValType&& k1 = x * cos(q_sol.q1[0]) + y * sin(q_sol.q1[0]);

				//k2 = x * cos(theta1_2) + y * sin(theta1_2); 
				ValType&& k2 = x * cos(q_sol.q1[1]) + y * sin(q_sol.q1[1]);

				//static ValType p = sqrt(229.840 * 229.840 + 27.200 * 27.200);
				//phi = atan2(27200, 229840);
				static ValType phi = atan2(0.027200, 0.229840);
				ValType&& t1 = (k1 - 0.03);
				ValType&& t2 = t1 * t1;
				ValType&& t3 = z * z; // z^2
				ValType&& t4 = (0.229840 * 0.229840 + 0.027200 * 0.027200); // p*p
				ValType&& t9 = t3 - 0.231444; // z^2 - 231.444
				ValType&& t5 = (t2 + t9); // (k1 - 0.03)^2 + z^2 - 231.444
				ValType&& t10 = t5 * t5; // ((k1 - 0.03)^2 + z^2 - 231.444)^2

				// q3:
				q_sol.q3[0] = static_cast<ValType>(atan2(t5, // q3-1
					sqrt(t4 - t10)) - phi);

				q_sol.q3[1] = static_cast<ValType>(atan2(t5,  // q3-2
					-sqrt(t4 - t10)) - phi);
				ValType&& t6 = (k2 - 0.03);
				ValType&& t7 = t6 * t6; // (k2 - 0.03)^2
				ValType&& t8 = (t7 + t9); // (k2 - 0.03)^2 + z^2 - 231.444
				ValType&& t11 = t8 * t8; //  ((k2 - 0.03)^2 + z^2 - 231.444)^2

				q_sol.q3[2] = static_cast<ValType>(atan2(t8,  // q3-3
					sqrt(t4 - t11)) - phi);
				q_sol.q3[3] = static_cast<ValType>(atan2(t8,  // q3-4
					-sqrt(t4 - t11)) - phi);
				// q2-1:
				//ValType&& t12 = 0.34 * cos(q_sol.q3[0]);
				//ValType&& t13 = 0.34 * sin(q_sol.q3[0]);
				ValType&& t14 = (0.34 * cos(q_sol.q3[0]) + 0.04); //(340*cos(q3-1)) + 0.04)
				ValType&& t15 = (0.34 * sin(q_sol.q3[0]) + 0.338); //(340*sin(q3-1)) + 0.338)
				ValType&& t16 = t14 * z; // ((340*cos(q3-1)) + 0.04)*z)
				ValType&& t17 = t15 * z; // ((340*sin(q3-1)) + 0.338)*z)
				ValType&& t18 = t14 * t1; // (340*cos(q3-1)) + 0.04)*(k1 - 0.03)
				ValType&& t19 = t15 * t1; // (340*sin(q3-1)) + 0.338)*(k1 - 0.03)

				q_sol.q2[0] = static_cast<ValType>(atan2(t16 + t19, -t17 + t18) - q_sol.q3[0]);
				// q2-2:
				//ValType&& t20 = 0.34 * cos(q_sol.q3[1]);
				//ValType&& t21 = 0.34 * sin(q_sol.q3[1]);
				ValType&& t22 = (0.34 * cos(q_sol.q3[1]) + 0.04); //(340*cos(q3-2)) + 0.04)
				ValType&& t23 = (0.34 * sin(q_sol.q3[1]) + 0.338); //(340*sin(q3-2)) + 0.338)
				ValType&& t24 = t22 * z; // ((340*cos(q3-2)) + 0.04)*z)
				ValType&& t25 = t23 * z; // ((340*sin(q3-2)) + 0.338)*z)
				ValType&& t26 = t22 * t1; // (340*cos(q3-2)) + 0.04)*(k1 - 0.03)
				ValType&& t27 = t23 * t1; // (340*sin(q3-2)) + 0.338)*(k1 - 0.03)

				q_sol.q2[1] = static_cast<ValType>(atan2(t24 + t27, -t25 + t26) - q_sol.q3[1]);
				// q2-3:
				ValType&& t28 = 0.34 * cos(q_sol.q3[2]) + 0.04;
				ValType&& t29 = 0.34 * sin(q_sol.q3[2]) + 0.338;
				ValType&& t30 = t28 * z; // (0.34 * cos(q_sol.q3[2]) + 0.04)*z
				ValType&& t31 = t29 * z; // (0.34 * sin(q_sol.q3[2]) + 0.338)*z
				ValType&& t32 = t28 * t6; // (0.34 * cos(q_sol.q3[2]) + 0.04)*(k2 - 0.03)
				ValType&& t33 = t29 * t6; // (0.34 * sin(q_sol.q3[2]) + 0.338)*(k2 - 0.03)

				q_sol.q2[2] = static_cast<ValType>(atan2(t30 + t33, -t31 + t32) - q_sol.q3[2]);
				// q2-4:
				ValType&& t34 = 0.34 * cos(q_sol.q3[3]) + 0.04;
				ValType&& t35 = 0.34 * sin(q_sol.q3[3]) + 0.338;
				ValType&& t36 = t34 * z; // (0.34 * cos(q_sol.q3[2]) + 0.04)*z
				ValType&& t37 = t35 * z; // (0.34 * sin(q_sol.q3[2]) + 0.338)*z
				ValType&& t38 = t34 * t6; // (0.34 * cos(q_sol.q3[2]) + 0.04)*(k2 - 0.03)
				ValType&& t39 = t35 * t6; // (0.34 * sin(q_sol.q3[2]) + 0.338)*(k2 - 0.03)

				q_sol.q2[3] = static_cast<ValType>(atan2(t36 + t39, -t37 + t38) - q_sol.q3[3]);
				// q5:
				ValType t40 = T06(0, 2); // T06(1, 3)
				ValType t41 = T06(1, 2); // T06(2, 3)
				ValType t42 = T06(2, 2); // T06(3, 3)
				ValType&& th231 = q_sol.q2[0] + q_sol.q3[0];
				ValType&& th232 = q_sol.q2[1] + q_sol.q3[1];
				ValType&& th233 = q_sol.q2[2] + q_sol.q3[2];
				ValType&& th234 = q_sol.q2[3] + q_sol.q3[3];
				ValType&& S231 = sin(th231);
				ValType&& S232 = sin(th232);
				ValType&& S233 = sin(th233);
				ValType&& S234 = sin(th234);
				ValType&& C231 = cos(th231);
				ValType&& C232 = cos(th232);
				ValType&& C233 = cos(th233);
				ValType&& C234 = cos(th234);
				ValType&& t43 = t40 * cos(q_sol.q1[0]);
				ValType&& t44 = t40 * cos(q_sol.q1[1]);
				ValType&& t45 = t40 * sin(q_sol.q1[0]);
				ValType&& t46 = t40 * sin(q_sol.q1[1]);
				ValType&& t47 = t41 * cos(q_sol.q1[0]);
				ValType&& t48 = t41 * cos(q_sol.q1[1]);
				ValType&& t49 = t41 * sin(q_sol.q1[0]);
				ValType&& t50 = t41 * sin(q_sol.q1[1]);

				ValType&& c51 = t43 * S231 + t49 * S231 - t42 * C231;
				ValType&& c53 = t43 * S232 + t49 * S232 - t42 * C232;
				ValType&& c55 = t44 * S233 + t50 * S233 - t42 * C233;
				ValType&& c57 = t44 * S234 + t50 * S234 - t42 * C234;

				q_sol.q5[0] = atan2(sqrt(1 - c51 * c51), c51);
				q_sol.q5[1] = -q_sol.q5[0];
				q_sol.q5[2] = atan2(sqrt(1 - c53 * c53), c53);
				q_sol.q5[3] = -q_sol.q5[2];
				q_sol.q5[4] = atan2(sqrt(1 - c55 * c55), c55);
				q_sol.q5[5] = -q_sol.q5[4];
				q_sol.q5[6] = atan2(sqrt(1 - c57 * c57), c57);
				q_sol.q5[7] = -q_sol.q5[6];

				// q4:
				ValType&& c41 = t43 * C231 + t49 * C231 + t42 * S231;
				ValType&& c43 = t43 * C232 + t49 * C232 + t42 * S232;
				ValType&& c45 = t44 * C233 + t50 * C233 + t42 * S233;
				ValType&& c47 = t44 * C234 + t50 * C234 + t42 * S234;
				// q4-1:
				q_sol.q4[0] = atan2(t45 - t47, c41);
				//if (!Check_WristSingular(q_sol.q5[0])) {
				//	q_sol.singularStates.at(0) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(0) = IK_SolutionState::Wrist;
				// q4-2:
				q_sol.q4[1] = q_sol.q4[0] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[1])) {
				//	q_sol.singularStates.at(1) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(1) = IK_SolutionState::Wrist;
				// q4-3:
				q_sol.q4[2] = atan2(t45 - t47, c43);
				//if (!Check_WristSingular(q_sol.q5[2])) {
				//	q_sol.singularStates.at(2) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(2) = IK_SolutionState::Wrist;
				// q4-4:
				q_sol.q4[3] = q_sol.q4[2] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[3])) {
				//	q_sol.singularStates.at(3) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(3) = IK_SolutionState::Wrist;
				// q4-5:
				q_sol.q4[4] = atan2(t46 - t48, c45);
				/*		if (!Check_WristSingular(q_sol.q5[4])) {
							q_sol.singularStates.at(4) = IK_SolutionState::None;

						}
						else
							q_sol.singularStates.at(4) = IK_SolutionState::Wrist;*/
							// q4-6:
				q_sol.q4[5] = q_sol.q4[4] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[5])) {
				//	q_sol.singularStates.at(5) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(5) = IK_SolutionState::Wrist;
				// q4-7:
				q_sol.q4[6] = atan2(t46 - t48, c47);
				//if (!Check_WristSingular(q_sol.q5[6])) {
				//	q_sol.singularStates.at(6) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(6) = IK_SolutionState::Wrist;
				// q4-8:
				q_sol.q4[7] = q_sol.q4[6] + M_PI;
				/*			if (!Check_WristSingular(q_sol.q5[7])) {
								q_sol.singularStates.at(7) = IK_SolutionState::None;

							}
							else
								q_sol.singularStates.at(7) = IK_SolutionState::Wrist;*/
								// q6:
				ValType t51 = T06(0, 1);  // T06(1, 2)
				ValType t52 = T06(1, 1);  // T06(2, 2)
				ValType t53 = T06(2, 1);  // T06(3, 2)
				ValType t54 = T06(0, 0);  // T06(1, 1)
				ValType t55 = T06(1, 0);  // T06(2, 1)
				ValType t56 = T06(2, 0);  // T06(3, 1)

				ValType&& t57 = t51 * cos(q_sol.q1[0]); // T06(1, 2)*cos(q1_1)
				ValType&& t58 = t51 * cos(q_sol.q1[1]); // T06(1, 2)*cos(q1_2)
				ValType&& t59 = -t54 * cos(q_sol.q1[0]); // T06(1, 1)*cos(q1_1)
				ValType&& t592 = -t54 * cos(q_sol.q1[1]); // T06(1, 1)*cos(q1_2)
				ValType&& t60 = t52 * sin(q_sol.q1[0]); // T06(2, 2)*sin(q1_1)
				ValType&& t61 = t52 * sin(q_sol.q1[1]); // T06(2, 2)*sin(q1_2)
				ValType&& t62 = -t55 * sin(q_sol.q1[0]); // T06(2, 1)*sin(q1_1)
				ValType&& t63 = -t55 * sin(q_sol.q1[1]); // T06(2, 1)*sin(q1_2)
				ValType&& t64 = -t53 * C231;   // T06(3, 2) * c231
				ValType&& t65 = t56 * C231;   // T06(3, 1) * c231
				ValType&& t66 = -t53 * C232;   // T06(3, 2) * c232
				ValType&& t67 = t56 * C232;   // T06(3, 1) * c232
				ValType&& t68 = -t53 * C233;   // T06(3, 2) * c233
				ValType&& t69 = t56 * C233;   // T06(3, 1) * c233
				ValType&& t610 = -t53 * C234;   // T06(3, 2) * c234
				ValType&& t611 = t56 * C234;   // T06(3, 1) * c234

				// q6:
				q_sol.q6[0] = atan2(t57 * S231 + t60 * S231 + t64, t59 * S231 + t62 * S231 + t65);
				q_sol.q6[1] = q_sol.q6[0] + M_PI;
				q_sol.q6[2] = atan2(t57 * S232 + t60 * S232 + t66, t59 * S232 + t62 * S232 + t67);
				q_sol.q6[3] = q_sol.q6[2] + M_PI;
				q_sol.q6[4] = atan2(t58 * S233 + t61 * S233 + t68, t592 * S233 + t63 * S233 + t69);
				q_sol.q6[5] = q_sol.q6[4] + M_PI;
				q_sol.q6[6] = atan2(t58 * S234 + t61 * S234 + t610, t592 * S234 + t63 * S234 + t611);
				q_sol.q6[7] = q_sol.q6[6] + M_PI;
				// ---------------------------------------------------------------------------
				//   q_ras := 2*PI - abs(NOT - POT)
				//   (1) Regular q solutions to the effective principle angle range:
				//	 (2) Check q solution whether they are over-traveling or not. 
				// q1:
				for (unsigned short i = 0; i < 2; ++i) {
					// (1): q_ras = pi/6, NOT = -75, POT = +225
					//      effective principle angle range: 
					//		(a) Negative: [0 , NOT] && [(NOT - q_ras) ,  (q1-2*PI-75)]
					//		(b) Postive:  [0 , POT] && [POT+q_ras]
					if (q_sol.q1[i] > deg2rad(285.0))
						q_sol.q1[i] -= 2.0 * M_PI;
					else if (q_sol.q1[i] < deg2rad(-135.0))
						q_sol.q1[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q1[i] <= deg2rad(-75.0) || q_sol.q1[i] >= deg2rad(255.0))
					//	for (unsigned short j = 0; j < 4; ++j)
					//		q_sol.singularStates.at(4.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q1[i] = rad2deg(q_sol.q1[i]);
				}
				// q2:
				for (unsigned short i = 0; i < 4; ++i) {
					// (1):
					if (q_sol.q2[i] > deg2rad(325.0))
						q_sol.q2[i] -= 2.0 * M_PI;
					else if (q_sol.q2[i] < deg2rad(-185.0))
						q_sol.q2[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q2[i] >= deg2rad(175.0) || q_sol.q2[i] <= deg2rad(-35.0))
					//	for (unsigned short j = 0; j < 2; ++j)
					//		q_sol.singularStates.at(2.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q2[i] = rad2deg(q_sol.q2[i]);
				}
				// q3:
				for (unsigned short i = 0; i < 4; ++i) {
					// (1):
					if (q_sol.q3[i] > deg2rad(305.0))
						q_sol.q3[i] -= 2.0 * M_PI;
					else if (q_sol.q3[i] < deg2rad(-175.0))
						q_sol.q3[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q3[i] >= deg2rad(185.0) || q_sol.q3[i] <= deg2rad(-55.0))
					//	for (unsigned short j = 0; j < 2; ++j)
					//		q_sol.singularStates.at(2.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q3[i] = rad2deg(q_sol.q3[i]);
				}
				// q4:
				for (unsigned short i = 0; i < 8; ++i) {
					// (1):
					if (q_sol.q4[i] > M_PI)
						q_sol.q4[i] -= 2.0 * M_PI;
					else if (q_sol.q4[i] < -M_PI)
						q_sol.q4[i] += 2.0 * M_PI;
					// (2):
					//if (abs(q_sol.q4[i]) >= deg2rad(190))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q4[i] = rad2deg(q_sol.q4[i]);
				}
				// q5:
				for (unsigned short i = 0; i < 8; ++i) {
					// (1):
					if (q_sol.q5[i] > deg2rad(245.0))
						q_sol.q5[i] -= 2.0 * M_PI;
					else if (q_sol.q5[i] < deg2rad(-245.0))
						q_sol.q5[i] += 2.0 * M_PI;
					// (2):
					//if (abs(q_sol.q5[i]) >= deg2rad(115))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q5[i] = rad2deg(q_sol.q5[i]);
				}
				// q6:
				for (unsigned short i = 0; i < 8; ++i) {
					if (q_sol.q6[i] > M_PI)
						q_sol.q6[i] -= 2.0 * M_PI;
					else if (q_sol.q6[i] < -M_PI)
						q_sol.q6[i] += 2.0 * M_PI;
					//if (abs(q_sol.q6[i]) >= deg2rad(360))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q6[i] = rad2deg(q_sol.q6[i]);
				}
			}
			//
			inline void RT605_IK_Sol8(ValType* PosVec, IK_Sol<ValType>& q_sol, bool _unit = UNIT_RAD) {
				Eigen::Affine3d&& T0_tcp =
					Eigen::Translation3d(Eigen::Vector3d(PosVec[0], PosVec[1], PosVec[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((PosVec[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((PosVec[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((PosVec[3]), Eigen::Vector3d::UnitX());
				Eigen::Matrix<ValType, 4, 4>&& T06 = T0_tcp.matrix() * Ttcp_6;
				double x{ T06(0, 3) }, y{ T06(1, 3) }, z{ T06(2, 3) };
				//std::cout << x << ", " << y << ", " << z << std::endl;
				// --------------------------- Compute the inverse kinematics: -------------------------------------------
				// Compute solution 1:
				q_sol.q1[0] = static_cast<ValType>(atan2(y, x)); // q1-1
				q_sol.q1[1] = q_sol.q1[0] + M_PI;
				//k1 = x * cos(theta1_1) + y * sin(theta1_1);
				ValType&& k1 = x * cos(q_sol.q1[0]) + y * sin(q_sol.q1[0]);

				//k2 = x * cos(theta1_2) + y * sin(theta1_2); 
				ValType&& k2 = x * cos(q_sol.q1[1]) + y * sin(q_sol.q1[1]);

				//static ValType p = sqrt(229.840 * 229.840 + 27.200 * 27.200);
				//phi = atan2(27200, 229840);
				static ValType phi = atan2(0.027200, 0.229840);
				ValType&& t1 = (k1 - 0.03);
				ValType&& t2 = t1 * t1;
				ValType&& t3 = z * z; // z^2
				ValType&& t4 = (0.229840 * 0.229840 + 0.027200 * 0.027200); // p*p
				ValType&& t9 = t3 - 0.231444; // z^2 - 231.444
				ValType&& t5 = (t2 + t9); // (k1 - 0.03)^2 + z^2 - 231.444
				ValType&& t10 = t5 * t5; // ((k1 - 0.03)^2 + z^2 - 231.444)^2

				// q3:
				q_sol.q3[0] = static_cast<ValType>(atan2(t5, // q3-1
					sqrt(t4 - t10)) - phi);

				q_sol.q3[1] = static_cast<ValType>(atan2(t5,  // q3-2
					-sqrt(t4 - t10)) - phi);
				ValType&& t6 = (k2 - 0.03);
				ValType&& t7 = t6 * t6; // (k2 - 0.03)^2
				ValType&& t8 = (t7 + t9); // (k2 - 0.03)^2 + z^2 - 231.444
				ValType&& t11 = t8 * t8; //  ((k2 - 0.03)^2 + z^2 - 231.444)^2

				q_sol.q3[2] = static_cast<ValType>(atan2(t8,  // q3-3
					sqrt(t4 - t11)) - phi);
				q_sol.q3[3] = static_cast<ValType>(atan2(t8,  // q3-4
					-sqrt(t4 - t11)) - phi);
				// q2-1:
				//ValType&& t12 = 0.34 * cos(q_sol.q3[0]);
				//ValType&& t13 = 0.34 * sin(q_sol.q3[0]);
				ValType&& t14 = (0.34 * cos(q_sol.q3[0]) + 0.04); //(340*cos(q3-1)) + 0.04)
				ValType&& t15 = (0.34 * sin(q_sol.q3[0]) + 0.338); //(340*sin(q3-1)) + 0.338)
				ValType&& t16 = t14 * z; // ((340*cos(q3-1)) + 0.04)*z)
				ValType&& t17 = t15 * z; // ((340*sin(q3-1)) + 0.338)*z)
				ValType&& t18 = t14 * t1; // (340*cos(q3-1)) + 0.04)*(k1 - 0.03)
				ValType&& t19 = t15 * t1; // (340*sin(q3-1)) + 0.338)*(k1 - 0.03)

				q_sol.q2[0] = static_cast<ValType>(atan2(t16 + t19, -t17 + t18) - q_sol.q3[0]);
				// q2-2:
				//ValType&& t20 = 0.34 * cos(q_sol.q3[1]);
				//ValType&& t21 = 0.34 * sin(q_sol.q3[1]);
				ValType&& t22 = (0.34 * cos(q_sol.q3[1]) + 0.04); //(340*cos(q3-2)) + 0.04)
				ValType&& t23 = (0.34 * sin(q_sol.q3[1]) + 0.338); //(340*sin(q3-2)) + 0.338)
				ValType&& t24 = t22 * z; // ((340*cos(q3-2)) + 0.04)*z)
				ValType&& t25 = t23 * z; // ((340*sin(q3-2)) + 0.338)*z)
				ValType&& t26 = t22 * t1; // (340*cos(q3-2)) + 0.04)*(k1 - 0.03)
				ValType&& t27 = t23 * t1; // (340*sin(q3-2)) + 0.338)*(k1 - 0.03)

				q_sol.q2[1] = static_cast<ValType>(atan2(t24 + t27, -t25 + t26) - q_sol.q3[1]);
				// q2-3:
				ValType&& t28 = 0.34 * cos(q_sol.q3[2]) + 0.04;
				ValType&& t29 = 0.34 * sin(q_sol.q3[2]) + 0.338;
				ValType&& t30 = t28 * z; // (0.34 * cos(q_sol.q3[2]) + 0.04)*z
				ValType&& t31 = t29 * z; // (0.34 * sin(q_sol.q3[2]) + 0.338)*z
				ValType&& t32 = t28 * t6; // (0.34 * cos(q_sol.q3[2]) + 0.04)*(k2 - 0.03)
				ValType&& t33 = t29 * t6; // (0.34 * sin(q_sol.q3[2]) + 0.338)*(k2 - 0.03)

				q_sol.q2[2] = static_cast<ValType>(atan2(t30 + t33, -t31 + t32) - q_sol.q3[2]);
				// q2-4:
				ValType&& t34 = 0.34 * cos(q_sol.q3[3]) + 0.04;
				ValType&& t35 = 0.34 * sin(q_sol.q3[3]) + 0.338;
				ValType&& t36 = t34 * z; // (0.34 * cos(q_sol.q3[2]) + 0.04)*z
				ValType&& t37 = t35 * z; // (0.34 * sin(q_sol.q3[2]) + 0.338)*z
				ValType&& t38 = t34 * t6; // (0.34 * cos(q_sol.q3[2]) + 0.04)*(k2 - 0.03)
				ValType&& t39 = t35 * t6; // (0.34 * sin(q_sol.q3[2]) + 0.338)*(k2 - 0.03)

				q_sol.q2[3] = static_cast<ValType>(atan2(t36 + t39, -t37 + t38) - q_sol.q3[3]);
				// q5:
				ValType t40 = T06(0, 2); // T06(1, 3)
				ValType t41 = T06(1, 2); // T06(2, 3)
				ValType t42 = T06(2, 2); // T06(3, 3)
				ValType&& th231 = q_sol.q2[0] + q_sol.q3[0];
				ValType&& th232 = q_sol.q2[1] + q_sol.q3[1];
				ValType&& th233 = q_sol.q2[2] + q_sol.q3[2];
				ValType&& th234 = q_sol.q2[3] + q_sol.q3[3];
				ValType&& S231 = sin(th231);
				ValType&& S232 = sin(th232);
				ValType&& S233 = sin(th233);
				ValType&& S234 = sin(th234);
				ValType&& C231 = cos(th231);
				ValType&& C232 = cos(th232);
				ValType&& C233 = cos(th233);
				ValType&& C234 = cos(th234);
				ValType&& t43 = t40 * cos(q_sol.q1[0]);
				ValType&& t44 = t40 * cos(q_sol.q1[1]);
				ValType&& t45 = t40 * sin(q_sol.q1[0]);
				ValType&& t46 = t40 * sin(q_sol.q1[1]);
				ValType&& t47 = t41 * cos(q_sol.q1[0]);
				ValType&& t48 = t41 * cos(q_sol.q1[1]);
				ValType&& t49 = t41 * sin(q_sol.q1[0]);
				ValType&& t50 = t41 * sin(q_sol.q1[1]);

				ValType&& c51 = t43 * S231 + t49 * S231 - t42 * C231;
				ValType&& c53 = t43 * S232 + t49 * S232 - t42 * C232;
				ValType&& c55 = t44 * S233 + t50 * S233 - t42 * C233;
				ValType&& c57 = t44 * S234 + t50 * S234 - t42 * C234;

				q_sol.q5[0] = atan2(sqrt(1 - c51 * c51), c51);
				q_sol.q5[1] = -q_sol.q5[0];
				q_sol.q5[2] = atan2(sqrt(1 - c53 * c53), c53);
				q_sol.q5[3] = -q_sol.q5[2];
				q_sol.q5[4] = atan2(sqrt(1 - c55 * c55), c55);
				q_sol.q5[5] = -q_sol.q5[4];
				q_sol.q5[6] = atan2(sqrt(1 - c57 * c57), c57);
				q_sol.q5[7] = -q_sol.q5[6];

				// q4:
				ValType&& c41 = t43 * C231 + t49 * C231 + t42 * S231;
				ValType&& c43 = t43 * C232 + t49 * C232 + t42 * S232;
				ValType&& c45 = t44 * C233 + t50 * C233 + t42 * S233;
				ValType&& c47 = t44 * C234 + t50 * C234 + t42 * S234;
				// q4-1:
				q_sol.q4[0] = atan2(t45 - t47, c41);
				//if (!Check_WristSingular(q_sol.q5[0])) {
				//	q_sol.singularStates.at(0) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(0) = IK_SolutionState::Wrist;
				// q4-2:
				q_sol.q4[1] = q_sol.q4[0] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[1])) {
				//	q_sol.singularStates.at(1) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(1) = IK_SolutionState::Wrist;
				// q4-3:
				q_sol.q4[2] = atan2(t45 - t47, c43);
				//if (!Check_WristSingular(q_sol.q5[2])) {
				//	q_sol.singularStates.at(2) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(2) = IK_SolutionState::Wrist;
				// q4-4:
				q_sol.q4[3] = q_sol.q4[2] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[3])) {
				//	q_sol.singularStates.at(3) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(3) = IK_SolutionState::Wrist;
				// q4-5:
				q_sol.q4[4] = atan2(t46 - t48, c45);
		/*		if (!Check_WristSingular(q_sol.q5[4])) {
					q_sol.singularStates.at(4) = IK_SolutionState::None;
					
				}
				else
					q_sol.singularStates.at(4) = IK_SolutionState::Wrist;*/
				// q4-6:
				q_sol.q4[5] = q_sol.q4[4] + M_PI;
				//if (!Check_WristSingular(q_sol.q5[5])) {
				//	q_sol.singularStates.at(5) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(5) = IK_SolutionState::Wrist;
				// q4-7:
				q_sol.q4[6] = atan2(t46 - t48, c47);
				//if (!Check_WristSingular(q_sol.q5[6])) {
				//	q_sol.singularStates.at(6) = IK_SolutionState::None;
				//	
				//}
				//else
				//	q_sol.singularStates.at(6) = IK_SolutionState::Wrist;
				// q4-8:
				q_sol.q4[7] = q_sol.q4[6] + M_PI;
	/*			if (!Check_WristSingular(q_sol.q5[7])) {
					q_sol.singularStates.at(7) = IK_SolutionState::None;
					
				}
				else
					q_sol.singularStates.at(7) = IK_SolutionState::Wrist;*/
				// q6:
				ValType t51 = T06(0, 1);  // T06(1, 2)
				ValType t52 = T06(1, 1);  // T06(2, 2)
				ValType t53 = T06(2, 1);  // T06(3, 2)
				ValType t54 = T06(0, 0);  // T06(1, 1)
				ValType t55 = T06(1, 0);  // T06(2, 1)
				ValType t56 = T06(2, 0);  // T06(3, 1)

				ValType&& t57 = t51 * cos(q_sol.q1[0]); // T06(1, 2)*cos(q1_1)
				ValType&& t58 = t51 * cos(q_sol.q1[1]); // T06(1, 2)*cos(q1_2)
				ValType&& t59 = -t54 * cos(q_sol.q1[0]); // T06(1, 1)*cos(q1_1)
				ValType&& t592 = -t54 * cos(q_sol.q1[1]); // T06(1, 1)*cos(q1_2)
				ValType&& t60 = t52 * sin(q_sol.q1[0]); // T06(2, 2)*sin(q1_1)
				ValType&& t61 = t52 * sin(q_sol.q1[1]); // T06(2, 2)*sin(q1_2)
				ValType&& t62 = -t55 * sin(q_sol.q1[0]); // T06(2, 1)*sin(q1_1)
				ValType&& t63 = -t55 * sin(q_sol.q1[1]); // T06(2, 1)*sin(q1_2)
				ValType&& t64 = -t53 * C231;   // T06(3, 2) * c231
				ValType&& t65 = t56 * C231;   // T06(3, 1) * c231
				ValType&& t66 = -t53 * C232;   // T06(3, 2) * c232
				ValType&& t67 = t56 * C232;   // T06(3, 1) * c232
				ValType&& t68 = -t53 * C233;   // T06(3, 2) * c233
				ValType&& t69 = t56 * C233;   // T06(3, 1) * c233
				ValType&& t610 = -t53 * C234;   // T06(3, 2) * c234
				ValType&& t611 = t56 * C234;   // T06(3, 1) * c234

				// q6:
				q_sol.q6[0] = atan2(t57 * S231 + t60 * S231 + t64, t59 * S231 + t62 * S231 + t65);
				q_sol.q6[1] = q_sol.q6[0] + M_PI;
				q_sol.q6[2] = atan2(t57 * S232 + t60 * S232 + t66, t59 * S232 + t62 * S232 + t67);
				q_sol.q6[3] = q_sol.q6[2] + M_PI;
				q_sol.q6[4] = atan2(t58 * S233 + t61 * S233 + t68, t592 * S233 + t63 * S233 + t69);
				q_sol.q6[5] = q_sol.q6[4] + M_PI;
				q_sol.q6[6] = atan2(t58 * S234 + t61 * S234 + t610, t592 * S234 + t63 * S234 + t611);
				q_sol.q6[7] = q_sol.q6[6] + M_PI;
				// ---------------------------------------------------------------------------
				//   q_ras := 2*PI - abs(NOT - POT)
				//   (1) Regular q solutions to the effective principle angle range:
				//	 (2) Check q solution whether they are over-traveling or not. 
				// q1:
				for (unsigned short i = 0; i < 2; ++i) {
					// (1): q_ras = pi/6, NOT = -75, POT = +225
					//      effective principle angle range: 
					//		(a) Negative: [0 , NOT] && [(NOT - q_ras) ,  (q1-2*PI-75)]
					//		(b) Postive:  [0 , POT] && [POT+q_ras]
					if (q_sol.q1[i] > deg2rad(285.0))
						q_sol.q1[i] -= 2.0 * M_PI;
					else if (q_sol.q1[i] < deg2rad(-135.0))
						q_sol.q1[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q1[i] <= deg2rad(-75.0) || q_sol.q1[i] >= deg2rad(255.0))
					//	for (unsigned short j = 0; j < 4; ++j)
					//		q_sol.singularStates.at(4.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q1[i] = rad2deg(q_sol.q1[i]);
				}
				// q2:
				for (unsigned short i = 0; i < 4; ++i) {
					// (1):
					if (q_sol.q2[i] > deg2rad(325.0))
						q_sol.q2[i] -= 2.0 * M_PI;
					else if (q_sol.q2[i] < deg2rad(-185.0))
						q_sol.q2[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q2[i] >= deg2rad(175.0) || q_sol.q2[i] <= deg2rad(-35.0))
					//	for (unsigned short j = 0; j < 2; ++j)
					//		q_sol.singularStates.at(2.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q2[i] = rad2deg(q_sol.q2[i]);
				}
				// q3:
				for (unsigned short i = 0; i < 4; ++i) {
					// (1):
					if (q_sol.q3[i] > deg2rad(305.0))
						q_sol.q3[i] -= 2.0 * M_PI;
					else if (q_sol.q3[i] < deg2rad(-175.0))
						q_sol.q3[i] += 2.0 * M_PI;
					// (2):
					//if (q_sol.q3[i] >= deg2rad(185.0) || q_sol.q3[i] <= deg2rad(-55.0))
					//	for (unsigned short j = 0; j < 2; ++j)
					//		q_sol.singularStates.at(2.0 * i + j) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q3[i] = rad2deg(q_sol.q3[i]);
				}
				// q4:
				for (unsigned short i = 0; i < 8; ++i) {
					// (1):
					if (q_sol.q4[i] > M_PI)
						q_sol.q4[i] -= 2.0 * M_PI;
					else if (q_sol.q4[i] < -M_PI)
						q_sol.q4[i] += 2.0 * M_PI;
					// (2):
					//if (abs(q_sol.q4[i]) >= deg2rad(190))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q4[i] = rad2deg(q_sol.q4[i]);
				}
				// q5:
				for (unsigned short i = 0; i < 8; ++i) {
					// (1):
					if (q_sol.q5[i] > deg2rad(245.0))
						q_sol.q5[i] -= 2.0 * M_PI;
					else if (q_sol.q5[i] < deg2rad(-245.0))
						q_sol.q5[i] += 2.0 * M_PI;
					// (2):
					//if (abs(q_sol.q5[i]) >= deg2rad(115))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q5[i] = rad2deg(q_sol.q5[i]);
				}
				// q6:
				for (unsigned short i = 0; i < 8; ++i) {
					if (q_sol.q6[i] > M_PI)
						q_sol.q6[i] -= 2.0 * M_PI;
					else if (q_sol.q6[i] < -M_PI)
						q_sol.q6[i] += 2.0 * M_PI;
					//if (abs(q_sol.q6[i]) >= deg2rad(360))
					//	q_sol.singularStates.at(i) = IK_SolutionState::OT;
					if (_unit == UNIT_DEGREE)
						q_sol.q6[i] = rad2deg(q_sol.q6[i]);
				}
			}
			//inline IK_SolutionState&& RT605_IK(std::array<ValType, 6>& PosVec, Robot_Postures posture, std::array<ValType, 6>& q, bool _unit = UNIT_RAD) {
			//	IK_Sol<ValType> q_sol;
			//	this->RT605_IK_Sol8(PosVec, q_sol, _unit);
			//	//q = q_sol(static_cast<unsigned short>(posture));
			//	q_sol.ReadSol(static_cast<unsigned short>(posture), q.data());
			//	for (auto& val : q)
			//		if (_unit == UNIT_DEGREE) {
			//			if (val < 0.0001)
			//				val = 0.0;
			//		}
			//		else {
			//			if (val < 1.7453e-06)
			//				val = 0.0;
			//		}
			//	return std::move(q_sol.singularStates.at(posture));
			//}
			// 
			inline void RT605_IK(std::array<ValType, 6>& PosVec, Robot_Postures posture, std::array<ValType, 6>& q, bool _unit = UNIT_RAD) {
				IK_Sol<ValType> q_sol;
				this->RT605_IK_Sol8(PosVec, q_sol, _unit);
				for (unsigned short i = 0; i < 6; ++i) {
					//q[i] = q_sol(static_cast<unsigned short>(posture)).at(i);
					q_sol.ReadSol(static_cast<unsigned short>(posture), q.data());
					if (_unit == UNIT_DEGREE) {
						if (q[i] < 0.0001)
							q[i] = 0.0;
					}
					else {
						if (q[i] < 1.7453e-06)
							q[i] = 0.0;
					}
				}
			}
			//
			inline void RT605_IK(ValType* PosVec, Robot_Postures posture, ValType* q, bool _unit = UNIT_RAD) {
				IK_Sol<ValType> q_sol;
				RT605_IK_Sol8(PosVec, q_sol, _unit);
				for (unsigned short i = 0; i < 6; ++i) {
					//q[i] = q_sol(static_cast<unsigned short>(posture)).at(i);
					q_sol.ReadSol(static_cast<unsigned short>(posture), q);
					if (_unit == UNIT_DEGREE) {
						if (q[i] < 0.0001)
							q[i] = 0.0;
					}
					else {
						if (q[i] < 1.7453e-06)
							q[i] = 0.0;
					}
				}
			}
			//
			void RT605_IK_Config2(ValType* PosVec, ValType* q) {
				Eigen::Affine3d&& T0_tcp =
					Eigen::Translation3d(Eigen::Vector3d(PosVec[0], PosVec[1], PosVec[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((PosVec[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((PosVec[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((PosVec[3]), Eigen::Vector3d::UnitX());
				Eigen::Matrix<double, 4, 4>&& T06 = T0_tcp.matrix() * Ttcp_6;
				double x{ T06(0, 3) }, y{ T06(1, 3) }, z{ T06(2, 3) };
				//std::cout << x << ", " << y << ", " << z << std::endl;
				// --------------------------- Compute the inverse kinematics: -------------------------------------------
				// Compute solution 1:
				q[0] = static_cast<double>(atan2(y, x)); // q1-1
				double&& k1 = x * cos(q[0]) + y * sin(q[0]);
				//double&& k2 = x * cos(q_sol.q1[1]) + y * sin(q_sol.q1[1]);
				double&& phi = atan2(0.027200, 0.229840);
				double&& t1 = (k1 - 0.03);
				double&& t2 = t1 * t1;
				double&& t3 = z * z; // z^2
				double&& t4 = (0.229840 * 0.229840 + 0.027200 * 0.027200); // p*p
				double&& t9 = t3 - 0.231444; // z^2 - 231.444
				double&& t5 = (t2 + t9); // (k1 - 0.03)^2 + z^2 - 231.444
				double&& t10 = t5 * t5; // ((k1 - 0.03)^2 + z^2 - 231.444)^2
				// q3:
				q[2] = static_cast<double>(atan2(t5, // q3-1
					sqrt(t4 - t10)) - phi);
				double&& t14 = (0.34 * cos(q[2]) + 0.04); //(340*cos(q3-1)) + 0.04)
				double&& t15 = (0.34 * sin(q[2]) + 0.338); //(340*sin(q3-1)) + 0.338)
				double&& t16 = t14 * z; // ((340*cos(q3-1)) + 0.04)*z)
				double&& t17 = t15 * z; // ((340*sin(q3-1)) + 0.338)*z)
				double&& t18 = t14 * t1; // (340*cos(q3-1)) + 0.04)*(k1 - 0.03)
				double&& t19 = t15 * t1; // (340*sin(q3-1)) + 0.338)*(k1 - 0.03)
				q[1] = static_cast<double>(atan2(t16 + t19, -t17 + t18) - q[2]);	
				// q5:
				double t40 = T06(0, 2); // T06(1, 3)
				double t41 = T06(1, 2); // T06(2, 3)
				double t42 = T06(2, 2); // T06(3, 3)
				double&& th231 = q[1] + q[2];
				double&& S231 = sin(th231);
				double&& C231 = cos(th231);
				double&& t43 = t40 * cos(q[0]);
				double&& t45 = t40 * sin(q[0]);
				double&& t47 = t41 * cos(q[0]);
				double&& t49 = t41 * sin(q[0]);
				double&& c51 = t43 * S231 + t49 * S231 - t42 * C231;
				q[4] = -atan2(sqrt(1 - c51 * c51), c51);
				// q4:
				double&& c41 = t43 * C231 + t49 * C231 + t42 * S231;
				// q4-2:
				q[3] = atan2(t45 - t47, c41) + M_PI;
				// q6:
				double t51 = T06(0, 1);  // T06(1, 2)
				double t52 = T06(1, 1);  // T06(2, 2)
				double t53 = T06(2, 1);  // T06(3, 2)
				double t54 = T06(0, 0);  // T06(1, 1)
				double t55 = T06(1, 0);  // T06(2, 1)
				double t56 = T06(2, 0);  // T06(3, 1)
				double&& t57 = t51 * cos(q[0]); // T06(1, 2)*cos(q1_1)
				double&& t59 = -t54 * cos(q[0]); // T06(1, 1)*cos(q1_1)
				double&& t60 = t52 * sin(q[0]); // T06(2, 2)*sin(q1_1)
				double&& t62 = -t55 * sin(q[0]); // T06(2, 1)*sin(q1_1)
				double&& t64 = -t53 * C231;   // T06(3, 2) * c231
				double&& t65 = t56 * C231;   // T06(3, 1) * c231
				// q6:
				q[5] = atan2(t57 * S231 + t60 * S231 + t64, t59 * S231 + t62 * S231 + t65) + M_PI; //
				// ---------------------------------------------------------------------------
				//   q_ras := 2*PI - abs(NOT - POT)
				//   (1) Regular q solutions to the effective principle angle range:
				//	 (2) Check q solution whether they are over-traveling or not. 
			// q1:
					// (1): q_ras = pi/6, NOT = -75, POT = +225
					//      effective principle angle range: 
					//		(a) Negative: [0 , NOT] && [(NOT - q_ras) ,  (q1-2*PI-75)]
					//		(b) Postive:  [0 , POT] && [POT+q_ras]
					if (q[0] > deg2rad(285.0))
						q[0] -= 2.0 * M_PI;
					else if (q[0] < deg2rad(-135.0))
						q[0] += 2.0 * M_PI;
				// q2:
					// (1):
					if (q[1] > deg2rad(325.0))
						q[1] -= 2.0 * M_PI;
					else if (q[1] < deg2rad(-185.0))
						q[1] += 2.0 * M_PI;
				// q3:
					// (1):
					if (q[2] > deg2rad(305.0))
						q[2] -= 2.0 * M_PI;
					else if (q[2] < deg2rad(-175.0))
						q[2] += 2.0 * M_PI;
				// q4:
					// (1):
					if (q[3] > M_PI)
						q[3] -= 2.0 * M_PI;
					else if (q[3] < -M_PI)
						q[3] += 2.0 * M_PI;
				// q5:
					// (1):
					if (q[4] > deg2rad(245.0))
						q[4] -= 2.0 * M_PI;
					else if (q[4] < deg2rad(-245.0))
						q[4] += 2.0 * M_PI;
				// q6:
					if (q[5] > M_PI)
						q[5] -= 2.0 * M_PI;
					else if (q[5] < -M_PI)
						q[5] += 2.0 * M_PI;
			}
			//
			void RT605_IK_Config1(ValType* PosVec, ValType* q) {
				Eigen::Affine3d&& T0_tcp =
					Eigen::Translation3d(Eigen::Vector3d(PosVec[0], PosVec[1], PosVec[2])) *
					// Rotate fixed Spatial angle X-Y-Z:
					Eigen::AngleAxisd((PosVec[5]), Eigen::Vector3d::UnitZ()) *
					Eigen::AngleAxisd((PosVec[4]), Eigen::Vector3d::UnitY()) *
					Eigen::AngleAxisd((PosVec[3]), Eigen::Vector3d::UnitX());
				Eigen::Matrix<double, 4, 4>&& T06 = T0_tcp.matrix() * Ttcp_6;
				double x{ T06(0, 3) }, y{ T06(1, 3) }, z{ T06(2, 3) };
				//std::cout << x << ", " << y << ", " << z << std::endl;
				// --------------------------- Compute the inverse kinematics: -------------------------------------------
				// Compute solution 1:
				q[0] = static_cast<double>(atan2(y, x)); // q1-1
				double&& k1 = x * cos(q[0]) + y * sin(q[0]);
				//double&& k2 = x * cos(q_sol.q1[1]) + y * sin(q_sol.q1[1]);
				double&& phi = atan2(0.027200, 0.229840);
				double&& t1 = (k1 - 0.03);
				double&& t2 = t1 * t1;
				double&& t3 = z * z; // z^2
				double&& t4 = (0.229840 * 0.229840 + 0.027200 * 0.027200); // p*p
				double&& t9 = t3 - 0.231444; // z^2 - 231.444
				double&& t5 = (t2 + t9); // (k1 - 0.03)^2 + z^2 - 231.444
				double&& t10 = t5 * t5; // ((k1 - 0.03)^2 + z^2 - 231.444)^2
				// q3:
				q[2] = static_cast<double>(atan2(t5, // q3-1
					sqrt(t4 - t10)) - phi);
				double&& t14 = (0.34 * cos(q[2]) + 0.04); //(340*cos(q3-1)) + 0.04)
				double&& t15 = (0.34 * sin(q[2]) + 0.338); //(340*sin(q3-1)) + 0.338)
				double&& t16 = t14 * z; // ((340*cos(q3-1)) + 0.04)*z)
				double&& t17 = t15 * z; // ((340*sin(q3-1)) + 0.338)*z)
				double&& t18 = t14 * t1; // (340*cos(q3-1)) + 0.04)*(k1 - 0.03)
				double&& t19 = t15 * t1; // (340*sin(q3-1)) + 0.338)*(k1 - 0.03)
				q[1] = static_cast<double>(atan2(t16 + t19, -t17 + t18) - q[2]);
				// q5:
				double t40 = T06(0, 2); // T06(1, 3)
				double t41 = T06(1, 2); // T06(2, 3)
				double t42 = T06(2, 2); // T06(3, 3)
				double&& th231 = q[1] + q[2];
				double&& S231 = sin(th231);
				double&& C231 = cos(th231);
				double&& t43 = t40 * cos(q[0]);
				double&& t45 = t40 * sin(q[0]);
				double&& t47 = t41 * cos(q[0]);
				double&& t49 = t41 * sin(q[0]);
				double&& c51 = t43 * S231 + t49 * S231 - t42 * C231;
				q[4] = atan2(sqrt(1 - c51 * c51), c51);
				// q4:
				double&& c41 = t43 * C231 + t49 * C231 + t42 * S231;
				// q4-1:
				q[3] = atan2(t45 - t47, c41);
				// q6:
				double t51 = T06(0, 1);  // T06(1, 2)
				double t52 = T06(1, 1);  // T06(2, 2)
				double t53 = T06(2, 1);  // T06(3, 2)
				double t54 = T06(0, 0);  // T06(1, 1)
				double t55 = T06(1, 0);  // T06(2, 1)
				double t56 = T06(2, 0);  // T06(3, 1)
				double&& t57 = t51 * cos(q[0]); // T06(1, 2)*cos(q1_1)
				double&& t59 = -t54 * cos(q[0]); // T06(1, 1)*cos(q1_1)
				double&& t60 = t52 * sin(q[0]); // T06(2, 2)*sin(q1_1)
				double&& t62 = -t55 * sin(q[0]); // T06(2, 1)*sin(q1_1)
				double&& t64 = -t53 * C231;   // T06(3, 2) * c231
				double&& t65 = t56 * C231;   // T06(3, 1) * c231
				// q6:
				q[5] = atan2(t57 * S231 + t60 * S231 + t64, t59 * S231 + t62 * S231 + t65); //
				// ---------------------------------------------------------------------------
				//   q_ras := 2*PI - abs(NOT - POT)
				//   (1) Regular q solutions to the effective principle angle range:
				//	 (2) Check q solution whether they are over-traveling or not. 
			// q1:
					// (1): q_ras = pi/6, NOT = -75, POT = +225
					//      effective principle angle range: 
					//		(a) Negative: [0 , NOT] && [(NOT - q_ras) ,  (q1-2*PI-75)]
					//		(b) Postive:  [0 , POT] && [POT+q_ras]
				if (q[0] > deg2rad(285.0))
					q[0] -= 2.0 * M_PI;
				else if (q[0] < deg2rad(-135.0))
					q[0] += 2.0 * M_PI;
				// q2:
					// (1):
				if (q[1] > deg2rad(325.0))
					q[1] -= 2.0 * M_PI;
				else if (q[1] < deg2rad(-185.0))
					q[1] += 2.0 * M_PI;
				// q3:
					// (1):
				if (q[2] > deg2rad(305.0))
					q[2] -= 2.0 * M_PI;
				else if (q[2] < deg2rad(-175.0))
					q[2] += 2.0 * M_PI;
				// q4:
					// (1):
				if (q[3] > M_PI)
					q[3] -= 2.0 * M_PI;
				else if (q[3] < -M_PI)
					q[3] += 2.0 * M_PI;
				// q5:
					// (1):
				if (q[4] > deg2rad(245.0))
					q[4] -= 2.0 * M_PI;
				else if (q[4] < deg2rad(-245.0))
					q[4] += 2.0 * M_PI;
				// q6:
				if (q[5] > M_PI)
					q[5] -= 2.0 * M_PI;
				else if (q[5] < -M_PI)
					q[5] += 2.0 * M_PI;
			}
			//
			/*
			*  1. q: 為了計算往Tool 坐標系下的相對位移量相對於WS坐標系下的移動量，必須知道當前各軸關節的姿態
			*  2. tool_rel: 輸入往Tool 坐標系下的相對位移量
			*  3. robot_ref_tcp_vec: 回傳計算結果，往Tool 坐標系下的相對位移量，相對於WS坐標系下的位置
			*/
			inline void PTP_Tool_XYZ(ValType* q, ValType* tool_rel, ValType* robot_ref_tcp_vec)
			{
				double t10;
				double t109;
				double t11;
				double t110;
				double t117;
				double t119;
				double t12;
				double t120;
				double t122;
				double t125;
				double t129;
				double t13;
				double t131;
				double t14;
				double t146;
				double t148;
				double t15;
				double t153;
				double t154;
				double t158;
				double t159;
				double t16;
				double t164;
				double t17;
				double t173;
				double t174;
				double t175;
				double t18;
				double t182;
				double t19;
				double t2;
				double t20;
				double t208;
				double t21;
				double t22;
				double t23;
				double t24;
				double t25;
				double t26;
				double t27;
				double t28;
				double t29;
				double t3;
				double t30;
				double t31;
				double t33;
				double t34;
				double t36;
				double t37;
				double t39;
				double t4;
				double t40;
				double t42;
				double t43;
				double t44;
				double t45;
				double t48;
				double t49;
				double t5;
				double t52;
				double t53;
				double t54;
				double t55;
				double t56;
				double t59;
				double t6;
				double t60;
				double t7;
				double t70;
				double t71;
				double t72;
				double t73;
				double t74;
				double t75;
				double t76;
				double t77;
				double t78;
				double t79;
				double t8;
				double t81;
				double t83;
				double t85;
				double t9;
				double t92;
				double t99;

				t2 = cos(q[0]);
				t3 = cos(q[1]);
				t4 = cos(q[2]);
				t5 = cos(q[3]);
				t6 = cos(q[4]);
				t7 = cos(q[5]);
				t8 = cos(kinePrt.refFrame_to_workspace[3]);
				t9 = cos(kinePrt.refFrame_to_workspace[4]);
				t10 = cos(kinePrt.refFrame_to_workspace[5]);
				t11 = cos(kinePrt.endFrame_to_tcp[3]);
				t12 = cos(kinePrt.endFrame_to_tcp[4]);
				t13 = cos(kinePrt.endFrame_to_tcp[5]);
				t14 = sin(q[0]);
				t15 = sin(q[1]);
				t16 = sin(q[2]);
				t17 = sin(q[3]);
				t18 = sin(q[4]);
				t19 = sin(q[5]);
				t20 = sin(kinePrt.refFrame_to_workspace[3]);
				t21 = sin(kinePrt.refFrame_to_workspace[4]);
				t22 = sin(kinePrt.refFrame_to_workspace[5]);
				t23 = sin(kinePrt.endFrame_to_tcp[3]);
				t24 = sin(kinePrt.endFrame_to_tcp[4]);
				t25 = sin(kinePrt.endFrame_to_tcp[5]);
				t26 = t8 * t8;
				t27 = t9 * t9;
				t28 = t10 * t10;
				t29 = t20 * t20;
				t30 = t21 * t21;
				t31 = t22 * t22;
				t33 = t3 * t4;
				t34 = t11 * t13;
				t36 = t3 * t16;
				t37 = t4 * t15;
				t39 = t11 * t25;
				t40 = t13 * t23;
				t42 = t15 * t16;
				t43 = t23 * t25;
				t44 = t8 * t21 * t22;
				t16 = t10 * t20;
				t45 = t16 * t21;
				t49 = t20 * t21 * t22;
				t4 = t8 * t10;
				t56 = t4 * t21;
				t48 = t14 * t42;
				t52 = t2 * t33;
				t53 = t2 * t36;
				t54 = t2 * t37;
				t55 = t14 * t33;
				t59 = t14 * t36;
				t60 = t14 * t37;
				t70 = t4 * t27;
				t71 = t4 * t30;
				t4 = t8 * t22;
				t72 = t4 * t27;
				t73 = t16 * t27;
				t74 = t4 * t30;
				t75 = t16 * t30;
				t4 = t20 * t22;
				t76 = t4 * t27;
				t77 = t4 * t30;
				t78 = t26 * t27;
				t79 = t27 * t28;
				t26 *= t30;
				t81 = t27 * t29;
				t83 = t28 * t30;
				t85 = t29 * t30;
				t92 = t36 + t37;
				t99 = 1.0 / (t27 + t30);
				t109 = t34 + t24 * t43;
				t110 = t43 + t24 * t34;
				t4 = t33 + -t42;
				t117 = t53 + t54;
				t34 = t59 + t60;
				t122 = t5 * t18 * t92;
				t146 = (t49 + t70) + t71;
				t148 = (t56 + t76) + t77;
				t153 = (-t44 + t73) + t75;
				t154 = (-t45 + t72) + t74;
				t119 = t39 + -(t24 * t40);
				t120 = t40 + -(t24 * t39);
				t125 = t6 * t4;
				t43 = t52 + t2 * -t42;
				t16 = t48 + -t55;
				t129 = t6 * t117;
				t131 = t6 * t34;
				t173 = 1.0 / (((t78 + t26) + t81) + t85);
				t174 = 1.0 / (((t79 + t27 * t31) + t83) + t30 * t31);
				t164 = t5 * t6 * t92 + t18 * t4;
				t208 = 1.0 /
					(((((((t31 * t78 + t28 * t26) + t29 * t79) + t31 * t26) + t31 * t81) +
						t29 * t83) +
						t31 * t85) +
						t28 * t78);
				t4 = t2 * t17 + t5 * t16;
				t26 = t14 * t17 + t5 * t43;
				t40 = t2 * t5 + -(t17 * t16);
				t16 = t5 * t14 + -(t17 * t43);
				t158 = t18 * t26;
				t159 = t18 * t4;
				t182 = t7 * t17 * t92 + t19 * t164;
				t36 = (((((t15 * 0.34 + t36 / 25.0) + t37 / 25.0) + t42 * 0.338) +
					-(t33 * 0.338)) +
					t122 * 0.0865) +
					-(t125 * 0.0865);
				t175 = t129 + t158;
				t43 = t18 * t34 + t6 * t4;
				t4 = t18 * t117 - t6 * t26;
				t39 = t19 * t16 + -t7 * t4;
				t34 = t7 * t16 + t19 * t4;
				t33 = ((((((t2 * 0.03 + t2 * t3 * 0.34) + t52 / 25.0) + -(t2 * t42 / 25.0)) +
					t53 * 0.338) +
					t54 * 0.338) +
					t129 * 0.0865) +
					t158 * 0.0865;
				t26 = t19 * t40 + t7 * t43;
				t15 = t9 * t10 * t174;
				t37 = ((((((t14 * 0.03 + t3 * t14 * 0.34) + t55 / 25.0) + -(t48 / 25.0)) +
					t59 * 0.338) +
					t60 * 0.338) +
					t131 * 0.0865) +
					-(t159 * 0.0865);
				t5 = t122 - t125;
				t29 = t131 - t159;
				t27 = t9 * t22 * t174;
				t30 = (-t21 * t99 * t5 + t15 * t175) + t27 * t29;
				t117 = t8 * t9 * t173;
				t129 = (t117 * t5 + t148 * t175 * t208) + -t153 * t208 * t29;
				t16 = t7 * t40 + -(t19 * t43);
				t4 = t17 * t19 * t92 - t7 * t164;
				t159 = (-t8 * t9 * t173 * t4 + t148 * t39 * t208) + t153 * t26 * t208;
				t83 = t9 * t20 * t173;
				t85 = (t83 * t4 + t146 * t26 * t208) + t154 * t39 * t208;
				t78 = (t83 * t182 + t146 * t16 * t208) + t154 * t34 * t208;
				t79 = (-(t117 * t182) + t148 * t34 * t208) + t153 * t16 * t208;
				t81 = t21 * t99;
				t40 = (t81 * t182 + t15 * t34) + -(t27 * t16);
				t158 = kinePrt.refFrame_to_workspace[2] * t21;
				t39 = (-(t27 * t26) + t15 * t39) + t81 * t4;
				t34 = t12 * t13;
				t43 = t12 * t25;
				t26 = t11 * t12;
				t16 = t12 * t23;
				robot_ref_tcp_vec[0] =
					((((((((tool_rel[0] * ((-t24 * t30 + t43 * t40) + t34 * t39) -
						t174 * (((kinePrt.refFrame_to_workspace[0] * t9 * t10 + kinePrt.refFrame_to_workspace[1] * t9 * t22) - t158 * t28) -
							t158 * t31)) +
						t30 * kinePrt.endFrame_to_tcp[2]) +
						t40 * kinePrt.endFrame_to_tcp[1]) +
						kinePrt.endFrame_to_tcp[0] * t39) +
						tool_rel[2] * ((-t120 * t40 + t110 * t39) + t26 * t30)) +
						tool_rel[1] * ((t109 * t40 - t119 * t39) + t16 * t30)) -
						t81 * t36) +
						t15 * t33) +
					t27 * t37;
				t158 = kinePrt.refFrame_to_workspace[2] * t9 * t20;
				t4 = t146 * t208;
				t39 = (-(t154 * t175 * t208) + t4 * t29) + t83 * t5;
				robot_ref_tcp_vec[1] =
					((((((((-tool_rel[0] * ((t24 * t39 + t34 * t85) + t43 * t78) -
						t208 * (((((((kinePrt.refFrame_to_workspace[0] * t45 + kinePrt.refFrame_to_workspace[1] * t49) + kinePrt.refFrame_to_workspace[1] * t70) -
							kinePrt.refFrame_to_workspace[0] * t72) +
							kinePrt.refFrame_to_workspace[1] * t71) -
							kinePrt.refFrame_to_workspace[0] * t74) +
							t158 * t28) +
							t158 * t31)) -
						t85 * kinePrt.endFrame_to_tcp[0]) -
						t78 * kinePrt.endFrame_to_tcp[1]) +
						kinePrt.endFrame_to_tcp[2] * t39) +
						tool_rel[2] * ((-t110 * t85 + t120 * t78) + t26 * t39)) +
						tool_rel[1] * ((-t109 * t78 + t119 * t85) + t16 * t39)) +
						t4 * t37) -
						t154 * t208 * t33) +
					t83 * t36;
				t158 = kinePrt.refFrame_to_workspace[2] * t8 * t9;
				robot_ref_tcp_vec[2] =
					((((((((-t208 * (((((((kinePrt.refFrame_to_workspace[1] * t44 + kinePrt.refFrame_to_workspace[0] * t56) - kinePrt.refFrame_to_workspace[1] * t73) +
						kinePrt.refFrame_to_workspace[0] * t76) -
						kinePrt.refFrame_to_workspace[1] * t75) +
						kinePrt.refFrame_to_workspace[0] * t77) +
						t158 * t28) +
						t158 * t31) +
						t159 * kinePrt.endFrame_to_tcp[0]) +
						t129 * kinePrt.endFrame_to_tcp[2]) +
						t79 * kinePrt.endFrame_to_tcp[1]) +
						tool_rel[2] * ((t110 * t159 - t120 * t79) + t26 * t129)) +
						tool_rel[1] * ((t109 * t79 - t119 * t159) + t16 * t129)) +
						tool_rel[0] * ((-t24 * t129 + t34 * t159) + t43 * t79)) +
						t148 * t208 * t33) -
						t153 * t208 * t37) +
					t117 * t36;
			}
			//
			inline void PTP_Tool_OrientationRx(ValType* q, ValType tool_rel, ValType* robot_ref_tcp_vec)
			{

				double t125_tmp;
				double t127_tmp;
				double t12_tmp;
				double t130_tmp;
				double t139_tmp;
				double t13_tmp;
				double t140_tmp;
				double t141_tmp;
				double t142_tmp;
				double t143_tmp;
				double t147_tmp;
				double t148_tmp;
				double t150_tmp;
				double t151_tmp;
				double t167;
				double t168_tmp;
				double t169_tmp;
				double t170_tmp;
				double t171_tmp;
				double t172_tmp;
				double t173_tmp;
				double t174_tmp;
				double t175_tmp;
				double t187_tmp;
				double t200_tmp;
				double t20_tmp;
				double t213_tmp;
				double t215_tmp;
				double t21_tmp;
				double t223_tmp;
				double t224;
				double t224_tmp;
				double t22_tmp;
				double t239;
				double t245;
				double t248;
				double t251;
				double t253;
				double t255_tmp;
				double t256;
				double t257_tmp;
				double t258;
				double t26;
				double t265;
				double t266_tmp;
				double t267_tmp;
				double t268;
				double t27;
				double t28;
				double t282;
				double t283;
				double t285;
				double t29;
				double t297;
				double t300;
				double t301;
				double t306;
				double t309;
				double t30_tmp;
				double t311;
				double t312;
				double t313;
				double t314;
				double t31_tmp;
				double t322;
				double t328;
				double t330;
				double t332;
				double t33_tmp;
				double t344;
				double t345;
				double t34_tmp;
				double t350;
				double t352;
				double t355;
				double t357;
				double t359;
				double t35_tmp;
				double t36_tmp;
				double t37_tmp;
				double t38_tmp;
				double t39;
				double t40;
				double t41;
				double t42;
				double t61_tmp;
				double t62_tmp;
				double t63_tmp;
				double t64_tmp;
				double t65_tmp;
				double t66_tmp;
				double t68_tmp;
				double t69;
				double t71_tmp;
				double t74;
				double t75;
				double t77_tmp;
				double t78;
				double t8_tmp;
				double t99_tmp;
				double t9_tmp;
				t8_tmp = cos(q[0]);
				t9_tmp = cos(q[1]);
				t283 = cos(q[2]);
				t344 = cos(q[3]);
				t12_tmp = cos(q[4]);
				t13_tmp = cos(q[5]);
				t20_tmp = cos(kinePrt.refFrame_to_workspace[3]);
				t21_tmp = cos(kinePrt.refFrame_to_workspace[4]);
				t22_tmp = cos(kinePrt.refFrame_to_workspace[5]);
				t26 = cos(kinePrt.endFrame_to_tcp[3]);
				t27 = cos(kinePrt.endFrame_to_tcp[4]);
				t28 = cos(kinePrt.endFrame_to_tcp[5]);
				t29 = cos(tool_rel);
				t30_tmp = sin(q[0]);
				t31_tmp = sin(q[1]);
				t248 = sin(q[2]);
				t33_tmp = sin(q[3]);
				t34_tmp = sin(q[4]);
				t35_tmp = sin(q[5]);
				t36_tmp = sin(kinePrt.refFrame_to_workspace[3]);
				t37_tmp = sin(kinePrt.refFrame_to_workspace[4]);
				t38_tmp = sin(kinePrt.refFrame_to_workspace[5]);
				t39 = sin(kinePrt.endFrame_to_tcp[3]);
				t40 = sin(kinePrt.endFrame_to_tcp[4]);
				t41 = sin(kinePrt.endFrame_to_tcp[5]);
				t42 = sin(tool_rel);
				t61_tmp = t20_tmp * t20_tmp;
				t62_tmp = t21_tmp * t21_tmp;
				t63_tmp = t22_tmp * t22_tmp;
				t64_tmp = t36_tmp * t36_tmp;
				t65_tmp = t37_tmp * t37_tmp;
				t66_tmp = t38_tmp * t38_tmp;
				t68_tmp = t9_tmp * t283;
				t69 = t26 * t28;
				t71_tmp = t9_tmp * t248;
				t355 = t283 * t31_tmp;
				t74 = t26 * t41;
				t75 = t28 * t39;
				t77_tmp = t31_tmp * t248;
				t78 = t39 * t41;
				t345 = t8_tmp * t33_tmp;
				t99_tmp = t344 * t30_tmp;
				t125_tmp = t61_tmp * t62_tmp;
				t312 = t62_tmp * t63_tmp;
				t127_tmp = t61_tmp * t65_tmp;
				t268 = t62_tmp * t64_tmp;
				t130_tmp = t63_tmp * t65_tmp;
				t285 = t64_tmp * t65_tmp;
				t283 = t20_tmp * t22_tmp;
				t143_tmp = t283 * t37_tmp;
				t147_tmp = t20_tmp * t37_tmp * t38_tmp;
				t248 = t22_tmp * t36_tmp;
				t148_tmp = t248 * t37_tmp;
				t150_tmp = t36_tmp * t37_tmp * t38_tmp;
				t151_tmp = t71_tmp + t355;
				t139_tmp = t8_tmp * t68_tmp;
				t140_tmp = t8_tmp * t71_tmp;
				t141_tmp = t8_tmp * t355;
				t142_tmp = t30_tmp * t68_tmp;
				t359 = t30_tmp * t71_tmp;
				t328 = t30_tmp * t355;
				t167 = 1.0 / (t62_tmp + t65_tmp);
				t168_tmp = t283 * t62_tmp;
				t169_tmp = t283 * t65_tmp;
				t283 = t20_tmp * t38_tmp;
				t170_tmp = t283 * t62_tmp;
				t171_tmp = t248 * t62_tmp;
				t172_tmp = t283 * t65_tmp;
				t173_tmp = t248 * t65_tmp;
				t283 = t36_tmp * t38_tmp;
				t174_tmp = t283 * t62_tmp;
				t175_tmp = t283 * t65_tmp;
				t187_tmp = t68_tmp + -t77_tmp;
				t352 = t140_tmp + t141_tmp;
				t200_tmp = t359 + t328;
				t213_tmp = t344 * t34_tmp;
				t215_tmp = t33_tmp * t35_tmp;
				t255_tmp = (t150_tmp + t168_tmp) + t169_tmp;
				t257_tmp = (t143_tmp + t174_tmp) + t175_tmp;
				t266_tmp = (-t147_tmp + t171_tmp) + t173_tmp;
				t267_tmp = (-t148_tmp + t170_tmp) + t172_tmp;
				t223_tmp = t139_tmp + t8_tmp * -t77_tmp;
				t224_tmp = t30_tmp * t77_tmp;
				t224 = t224_tmp + -t142_tmp;
				t245 = t213_tmp * t151_tmp;
				t300 = 1.0 / (((t125_tmp + t127_tmp) + t268) + t285);
				t61_tmp = t62_tmp * t66_tmp;
				t297 = t65_tmp * t66_tmp;
				t301 = 1.0 / (((t312 + t61_tmp) + t130_tmp) + t297);
				t239 = t12_tmp * t187_tmp;
				t248 = t142_tmp + t30_tmp * -t77_tmp;
				t251 = t12_tmp * t352;
				t253 = t12_tmp * t200_tmp;
				t265 = t344 * t12_tmp;
				t282 = t265 * t151_tmp + t34_tmp * t187_tmp;
				t313 = 1.0 / (((t125_tmp + t127_tmp) + t268) + t285);
				t314 = 1.0 / (((t312 + t61_tmp) + t130_tmp) + t297);
				t350 =
					1.0 / (((((((t66_tmp * t125_tmp + t63_tmp * t127_tmp) + t64_tmp * t312) +
						t66_tmp * t127_tmp) +
						t66_tmp * t268) +
						t64_tmp * t130_tmp) +
						t66_tmp * t285) +
						t63_tmp * t125_tmp);
				t256 = t345 + t344 * t224;
				t283 = t30_tmp * t33_tmp;
				t258 = t283 + t344 * t223_tmp;
				t306 = t34_tmp * t187_tmp + t265 * t151_tmp;
				t309 = t239 + -t245;
				t357 = 1.0 /
					(((((((t63_tmp * t125_tmp + t66_tmp * t125_tmp) + t63_tmp * t127_tmp) +
						t64_tmp * t312) +
						t66_tmp * t127_tmp) +
						t66_tmp * t268) +
						t64_tmp * t130_tmp) +
						t66_tmp * t285);
				t61_tmp = t8_tmp * t344;
				t265 = t61_tmp + -(t33_tmp * t224);
				t268 = t99_tmp + -(t33_tmp * t223_tmp);
				t283 += t344 * t223_tmp;
				t285 = t61_tmp + t33_tmp * t248;
				t61_tmp = t13_tmp * t33_tmp;
				t322 = t61_tmp * t151_tmp + t35_tmp * t282;
				t297 = t34_tmp * t283;
				t311 = t12_tmp * t352 + t34_tmp * t258;
				t312 = t34_tmp * t200_tmp + t12_tmp * t256;
				t248 = t345 - t344 * t248;
				t330 = t253 + -t34_tmp * t248;
				t332 = t61_tmp * t151_tmp + t35_tmp * t306;
				t355 =
					(((((t31_tmp * 0.34 + t71_tmp / 25.0) + t355 / 25.0) + t77_tmp * 0.338) +
						-(t68_tmp * 0.338)) +
						-(t239 * 0.0865)) +
					t245 * 0.0865;
				t359 = ((((((t30_tmp * 0.03 + t9_tmp * t30_tmp * 0.34) + t142_tmp / 25.0) +
					-(t224_tmp / 25.0)) +
					t359 * 0.338) +
					t328 * 0.338) +
					t253 * 0.0865) +
					t34_tmp * t248 * -0.0865;
				t328 = t251 + t297;
				t142_tmp = t34_tmp * t352 - t12_tmp * t283;
				t61_tmp = t34_tmp * t352 - t12_tmp * t258;
				t344 = t35_tmp * t268 + -t13_tmp * t61_tmp;
				t345 = t13_tmp * t268 + t35_tmp * t61_tmp;
				t61_tmp = t34_tmp * t200_tmp + t12_tmp * t248;
				t352 = t13_tmp * t285 + -t35_tmp * t61_tmp;
				t258 = t35_tmp * t285 + t13_tmp * t61_tmp;
				t253 = t35_tmp * t265 + t13_tmp * t312;
				t239 = ((((((t8_tmp * 0.03 + t8_tmp * t9_tmp * 0.34) + t139_tmp / 25.0) +
					-(t8_tmp * t77_tmp / 25.0)) +
					t140_tmp * 0.338) +
					t141_tmp * 0.338) +
					t251 * 0.0865) +
					t297 * 0.0865;
				t224_tmp = t20_tmp * t21_tmp;
				t61_tmp = t224_tmp * t300;
				t245 = t12_tmp * t200_tmp - t34_tmp * t256;
				t64_tmp = t213_tmp * t151_tmp - t12_tmp * t187_tmp;
				t285 = (t61_tmp * t64_tmp + t257_tmp * t311 * t350) + -t266_tmp * t350 * t245;
				t312 = t13_tmp * t265 + -(t35_tmp * t312);
				t125_tmp = t215_tmp * t151_tmp - t13_tmp * t282;
				t283 = (-t20_tmp * t21_tmp * t300 * t125_tmp + t257_tmp * t344 * t350) +
					t266_tmp * t253 * t350;
				t248 = (-(t61_tmp * t322) + t257_tmp * t345 * t350) + t266_tmp * t312 * t350;
				t268 = (t27 * t39 * t285 + (t69 + t40 * t78) * t248) +
					-((t74 + -(t40 * t75)) * t283);
				t224 = (t26 * t27 * t285 + (t78 + t40 * t69) * t283) +
					-((t75 + -(t40 * t74)) * t248);
				t130_tmp = t27 * t28;
				t61_tmp = t130_tmp * t283;
				t127_tmp = t27 * t41;
				t283 = t127_tmp * t248;
				t248 = (-t40 * t285 + t61_tmp) + t283;
				robot_ref_tcp_vec[3] =
					atan2(t29 * t268 + t42 * t224, t29 * t224 - t42 * t268);
				robot_ref_tcp_vec[4] =
					atan2((t40 * t285 - t61_tmp) - t283, sqrt(-(t248 * t248) + 1.0));
				t265 = t21_tmp * t36_tmp;
				t285 = t265 * t300;
				t283 = t21_tmp * t22_tmp;
				t297 = t283 * t301;
				t248 = t21_tmp * t38_tmp;
				t224 = t248 * t301;
				t268 = t37_tmp * t167;
				robot_ref_tcp_vec[5] = atan2(
					(-t40 * ((-t267_tmp * t311 * t350 + t255_tmp * t350 * t245) +
						t285 * t64_tmp) -
						t127_tmp *
						((t255_tmp * t312 * t350 + t267_tmp * t345 * t350) + t285 * t322)) -
					t130_tmp * ((t255_tmp * t253 * t350 + t267_tmp * t344 * t350) +
						t285 * t125_tmp),
					(-t40 * ((-t37_tmp * t167 * t64_tmp + t297 * t311) + t224 * t245) +
						t127_tmp * ((t268 * t322 + t297 * t345) - t224 * t312)) +
					t130_tmp * ((t268 * t125_tmp + t297 * t344) - t224 * t253));
				t285 = kinePrt.refFrame_to_workspace[2] * t37_tmp;
				t297 = t37_tmp * (1.0 / (t62_tmp + t65_tmp));
				t224 = t283 * t314;
				t268 = t248 * t314;
				t248 = t99_tmp - t33_tmp * t223_tmp;
				t61_tmp = t13_tmp * t248 + t35_tmp * t142_tmp;
				t248 = -t13_tmp * t142_tmp + t35_tmp * t248;
				t283 = t215_tmp * t151_tmp - t13_tmp * t306;
				robot_ref_tcp_vec[0] =
					(((((-t314 * (((kinePrt.refFrame_to_workspace[0] * t21_tmp * t22_tmp + kinePrt.refFrame_to_workspace[1] * t21_tmp * t38_tmp) -
						t285 * t63_tmp) -
						t285 * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t297 * t332 + t224 * t61_tmp) - t268 * t352)) +
						kinePrt.endFrame_to_tcp[2] * ((t297 * t309 + t224 * t328) + t268 * t330)) +
						kinePrt.endFrame_to_tcp[0] * ((t297 * t283 + t224 * t248) - t268 * t258)) -
						t297 * t355) +
						t224 * t239) +
					t268 * t359;
				t285 = kinePrt.refFrame_to_workspace[2] * t21_tmp * t36_tmp;
				t297 = t267_tmp * t357;
				t224 = t265 * t313;
				robot_ref_tcp_vec[1] =
					(((((-t357 * (((((((kinePrt.refFrame_to_workspace[0] * t148_tmp + kinePrt.refFrame_to_workspace[1] * t150_tmp) +
						kinePrt.refFrame_to_workspace[1] * t168_tmp) -
						kinePrt.refFrame_to_workspace[0] * t170_tmp) +
						kinePrt.refFrame_to_workspace[1] * t169_tmp) -
						kinePrt.refFrame_to_workspace[0] * t172_tmp) +
						t285 * t63_tmp) +
						t285 * t66_tmp) -
						kinePrt.endFrame_to_tcp[1] * ((t297 * t61_tmp + t255_tmp * t352 * t357) + t224 * t332)) -
						kinePrt.endFrame_to_tcp[2] * ((-t255_tmp * t330 * t357 + t267_tmp * t328 * t357) +
							t265 * t309 * t313)) -
						kinePrt.endFrame_to_tcp[0] * ((t297 * t248 + t255_tmp * t258 * t357) + t224 * t283)) +
						t255_tmp * t357 * t359) -
						t297 * t239) +
					t224 * t355;
				t285 = kinePrt.refFrame_to_workspace[2] * t20_tmp * t21_tmp;
				t297 = t257_tmp * t357;
				t224 = t224_tmp * t313;
				robot_ref_tcp_vec[2] =
					(((((-t357 * (((((((kinePrt.refFrame_to_workspace[0] * t143_tmp + kinePrt.refFrame_to_workspace[1] * t147_tmp) -
						kinePrt.refFrame_to_workspace[1] * t171_tmp) +
						kinePrt.refFrame_to_workspace[0] * t174_tmp) -
						kinePrt.refFrame_to_workspace[1] * t173_tmp) +
						kinePrt.refFrame_to_workspace[0] * t175_tmp) +
						t285 * t63_tmp) +
						t285 * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t297 * t61_tmp + t266_tmp * t352 * t357) - t224 * t332)) -
						kinePrt.endFrame_to_tcp[2] * ((-t257_tmp * t328 * t357 + t266_tmp * t330 * t357) +
							t224_tmp * t309 * t313)) +
						kinePrt.endFrame_to_tcp[0] * ((t297 * t248 + t266_tmp * t258 * t357) - t224 * t283)) +
						t297 * t239) -
						t266_tmp * t357 * t359) +
					t224 * t355;
			}
			//
			inline void PTP_Tool_OrientationRy(ValType* q, ValType tool_rel, ValType* robot_ref_tcp_vec)
			{
				double t11_tmp;
				double t121_tmp;
				double t12_tmp;
				double t134_tmp;
				double t135_tmp;
				double t136_tmp;
				double t137_tmp;
				double t138_tmp;
				double t13_tmp;
				double t140_tmp;
				double t142_tmp;
				double t143_tmp;
				double t145_tmp;
				double t146_tmp;
				double t162;
				double t163_tmp;
				double t164_tmp;
				double t165_tmp;
				double t166_tmp;
				double t167_tmp;
				double t168_tmp;
				double t169_tmp;
				double t170_tmp;
				double t180;
				double t180_tmp;
				double t181_tmp;
				double t193_tmp;
				double t194_tmp;
				double t195;
				double t206_tmp;
				double t208_tmp;
				double t20_tmp;
				double t216_tmp;
				double t217;
				double t21_tmp;
				double t22_tmp;
				double t232;
				double t238;
				double t241;
				double t244;
				double t246;
				double t248_tmp;
				double t249;
				double t250_tmp;
				double t251;
				double t258;
				double t259_tmp;
				double t26;
				double t260_tmp;
				double t261;
				double t27;
				double t275;
				double t276;
				double t278;
				double t28;
				double t29;
				double t290;
				double t293;
				double t294;
				double t294_tmp;
				double t299;
				double t302;
				double t304;
				double t305;
				double t306;
				double t307;
				double t316;
				double t31_tmp;
				double t322;
				double t324;
				double t328;
				double t329_tmp;
				double t339;
				double t33_tmp;
				double t340;
				double t341;
				double t342;
				double t34_tmp;
				double t353;
				double t355;
				double t357;
				double t358;
				double t35_tmp;
				double t366;
				double t367;
				double t368;
				double t36_tmp;
				double t37_tmp;
				double t385;
				double t38_tmp;
				double t39;
				double t40;
				double t41;
				double t42;
				double t62_tmp;
				double t63_tmp;
				double t65_tmp;
				double t66_tmp;
				double t68_tmp;
				double t73;
				double t75_tmp;
				double t76;
				double t8_tmp;
				double t92_tmp;
				double t95_tmp;
				double t9_tmp;
				t8_tmp = cos(q[0]);
				t9_tmp = cos(q[1]);
				t217 = cos(q[2]);
				t11_tmp = cos(q[3]);
				t12_tmp = cos(q[4]);
				t13_tmp = cos(q[5]);
				t20_tmp = cos(kinePrt.refFrame_to_workspace[3]);
				t21_tmp = cos(kinePrt.refFrame_to_workspace[4]);
				t22_tmp = cos(kinePrt.refFrame_to_workspace[5]);
				t26 = cos(kinePrt.endFrame_to_tcp[3]);
				t27 = cos(kinePrt.endFrame_to_tcp[4]);
				t28 = cos(kinePrt.endFrame_to_tcp[5]);
				t29 = cos(tool_rel);
				t357 = sin(q[0]);
				t31_tmp = sin(q[1]);
				t340 = sin(q[2]);
				t33_tmp = sin(q[3]);
				t34_tmp = sin(q[4]);
				t35_tmp = sin(q[5]);
				t36_tmp = sin(kinePrt.refFrame_to_workspace[3]);
				t37_tmp = sin(kinePrt.refFrame_to_workspace[4]);
				t38_tmp = sin(kinePrt.refFrame_to_workspace[5]);
				t39 = sin(kinePrt.endFrame_to_tcp[3]);
				t40 = sin(kinePrt.endFrame_to_tcp[4]);
				t41 = sin(kinePrt.endFrame_to_tcp[5]);
				t42 = sin(tool_rel);
				t276 = t20_tmp * t20_tmp;
				t62_tmp = t21_tmp * t21_tmp;
				t63_tmp = t22_tmp * t22_tmp;
				t290 = t36_tmp * t36_tmp;
				t65_tmp = t37_tmp * t37_tmp;
				t66_tmp = t38_tmp * t38_tmp;
				t68_tmp = t9_tmp * t217;
				t341 = t9_tmp * t340;
				t339 = t217 * t31_tmp;
				t73 = t28 * t39;
				t75_tmp = t31_tmp * t340;
				t76 = t39 * t41;
				t92_tmp = t8_tmp * t33_tmp;
				t95_tmp = t11_tmp * t357;
				t261 = t276 * t62_tmp;
				t121_tmp = t62_tmp * t63_tmp;
				t342 = t276 * t65_tmp;
				t305 = t62_tmp * t290;
				t258 = t63_tmp * t65_tmp;
				t316 = t290 * t65_tmp;
				t217 = t20_tmp * t22_tmp;
				t138_tmp = t217 * t37_tmp;
				t142_tmp = t20_tmp * t37_tmp * t38_tmp;
				t340 = t22_tmp * t36_tmp;
				t143_tmp = t340 * t37_tmp;
				t145_tmp = t36_tmp * t37_tmp * t38_tmp;
				t146_tmp = t341 + t339;
				t180_tmp = t26 * t28;
				t180 = t76 + t180_tmp * t40;
				t134_tmp = t8_tmp * t68_tmp;
				t135_tmp = t8_tmp * t341;
				t136_tmp = t8_tmp * t339;
				t137_tmp = t357 * t68_tmp;
				t140_tmp = t357 * t341;
				t367 = t357 * t339;
				t162 = 1.0 / (t62_tmp + t65_tmp);
				t163_tmp = t217 * t62_tmp;
				t164_tmp = t217 * t65_tmp;
				t217 = t20_tmp * t38_tmp;
				t165_tmp = t217 * t62_tmp;
				t166_tmp = t340 * t62_tmp;
				t167_tmp = t217 * t65_tmp;
				t168_tmp = t340 * t65_tmp;
				t217 = t36_tmp * t38_tmp;
				t169_tmp = t217 * t62_tmp;
				t170_tmp = t217 * t65_tmp;
				t181_tmp = t68_tmp + -t75_tmp;
				t193_tmp = t135_tmp + t136_tmp;
				t194_tmp = t140_tmp + t367;
				t195 = t73 + -(t26 * t40 * t41);
				t206_tmp = t11_tmp * t34_tmp;
				t208_tmp = t33_tmp * t35_tmp;
				t248_tmp = (t145_tmp + t163_tmp) + t164_tmp;
				t250_tmp = (t138_tmp + t169_tmp) + t170_tmp;
				t259_tmp = (-t142_tmp + t166_tmp) + t168_tmp;
				t260_tmp = (-t143_tmp + t165_tmp) + t167_tmp;
				t216_tmp = t134_tmp + t8_tmp * -t75_tmp;
				t385 = t357 * t75_tmp;
				t217 = t385 + -t137_tmp;
				t238 = t206_tmp * t146_tmp;
				t293 = 1.0 / (((t261 + t342) + t305) + t316);
				t278 = t62_tmp * t66_tmp;
				t294_tmp = t65_tmp * t66_tmp;
				t294 = 1.0 / (((t121_tmp + t278) + t258) + t294_tmp);
				t232 = t12_tmp * t181_tmp;
				t241 = t137_tmp + t357 * -t75_tmp;
				t244 = t12_tmp * t193_tmp;
				t246 = t12_tmp * t194_tmp;
				t340 = t11_tmp * t12_tmp;
				t275 = t340 * t146_tmp + t34_tmp * t181_tmp;
				t306 = 1.0 / (((t261 + t342) + t305) + t316);
				t307 = 1.0 / (((t121_tmp + t278) + t258) + t294_tmp);
				t353 = 1.0 / (((((((t66_tmp * t261 + t63_tmp * t342) + t290 * t121_tmp) +
					t66_tmp * t342) +
					t66_tmp * t305) +
					t290 * t258) +
					t66_tmp * t316) +
					t63_tmp * t261);
				t249 = t92_tmp + t11_tmp * t217;
				t276 = t357 * t33_tmp;
				t251 = t276 + t11_tmp * t216_tmp;
				t299 = t34_tmp * t181_tmp + t340 * t146_tmp;
				t302 = t232 + -t238;
				t366 = 1.0 / (((((((t63_tmp * t261 + t66_tmp * t261) + t63_tmp * t342) +
					t290 * t121_tmp) +
					t66_tmp * t342) +
					t66_tmp * t305) +
					t290 * t258) +
					t66_tmp * t316);
				t340 = t8_tmp * t11_tmp;
				t258 = t340 + -(t33_tmp * t217);
				t261 = t95_tmp + -(t33_tmp * t216_tmp);
				t276 += t11_tmp * t216_tmp;
				t278 = t340 + t33_tmp * t241;
				t340 = t13_tmp * t33_tmp;
				t316 = t340 * t146_tmp + t35_tmp * t275;
				t290 = t34_tmp * t276;
				t304 = t12_tmp * t193_tmp + t34_tmp * t251;
				t305 = t34_tmp * t194_tmp + t12_tmp * t249;
				t294_tmp = t92_tmp - t11_tmp * t241;
				t324 = t246 + -t34_tmp * t294_tmp;
				t328 = t340 * t146_tmp + t35_tmp * t299;
				t358 = (((((t31_tmp * 0.34 + t341 / 25.0) + t339 / 25.0) + t75_tmp * 0.338) +
					-(t68_tmp * 0.338)) +
					-(t232 * 0.0865)) +
					t238 * 0.0865;
				t368 = ((((((t357 * 0.03 + t9_tmp * t357 * 0.34) + t137_tmp / 25.0) +
					-(t385 / 25.0)) +
					t140_tmp * 0.338) +
					t367 * 0.338) +
					t246 * 0.0865) +
					t34_tmp * t294_tmp * -0.0865;
				t322 = t244 + t290;
				t329_tmp = t34_tmp * t193_tmp - t12_tmp * t276;
				t217 = t34_tmp * t193_tmp - t12_tmp * t251;
				t341 = t35_tmp * t261 + -t13_tmp * t217;
				t342 = t13_tmp * t261 + t35_tmp * t217;
				t217 = t34_tmp * t194_tmp + t12_tmp * t294_tmp;
				t355 = t13_tmp * t278 + -t35_tmp * t217;
				t357 = t35_tmp * t278 + t13_tmp * t217;
				t339 = t35_tmp * t258 + t13_tmp * t305;
				t68_tmp = t21_tmp * t22_tmp;
				t31_tmp = t68_tmp * t294;
				t367 = ((((((t8_tmp * 0.03 + t8_tmp * t9_tmp * 0.34) + t134_tmp / 25.0) +
					-(t8_tmp * t75_tmp / 25.0)) +
					t135_tmp * 0.338) +
					t136_tmp * 0.338) +
					t244 * 0.0865) +
					t290 * 0.0865;
				t137_tmp = t206_tmp * t146_tmp - t12_tmp * t181_tmp;
				t246 = t12_tmp * t194_tmp - t34_tmp * t249;
				t11_tmp = t21_tmp * t38_tmp;
				t92_tmp = t11_tmp * t294;
				t251 = (-t37_tmp * t162 * t137_tmp + t31_tmp * t304) + t92_tmp * t246;
				t140_tmp = t20_tmp * t21_tmp;
				t217 = t140_tmp * t293;
				t385 = (t217 * t137_tmp + t250_tmp * t304 * t353) + -t259_tmp * t353 * t246;
				t340 = t13_tmp * t258 + -(t35_tmp * t305);
				t241 = t208_tmp * t146_tmp - t13_tmp * t275;
				t290 = (-t20_tmp * t21_tmp * t293 * t241 + t250_tmp * t341 * t353) +
					t259_tmp * t339 * t353;
				t193_tmp = t21_tmp * t36_tmp;
				t294_tmp = t193_tmp * t293;
				t238 = (t294_tmp * t241 + t248_tmp * t339 * t353) + t260_tmp * t341 * t353;
				t232 = (t294_tmp * t316 + t248_tmp * t340 * t353) + t260_tmp * t342 * t353;
				t278 = (-(t217 * t316) + t250_tmp * t342 * t353) + t259_tmp * t340 * t353;
				t121_tmp = t37_tmp * t162;
				t261 = (t121_tmp * t316 + t31_tmp * t342) + -(t92_tmp * t340);
				t340 = t27 * t28;
				t258 = t27 * t41;
				t217 = (-(t40 * t385) + t340 * t290) + t258 * t278;
				t316 = t26 * t27;
				t276 = (t316 * t385 + t180 * t290) + -(t195 * t278);
				t342 = t42 * t276;
				t305 = t29 * t217 - t342;
				robot_ref_tcp_vec[3] = atan2(
					(t278 * (t180_tmp + t40 * t76) - t290 * (t26 * t41 - t40 * t73)) +
					t27 * t39 * t385,
					t29 * t276 + t42 * t217);
				robot_ref_tcp_vec[4] =
					atan2(-t29 * t217 + t342, sqrt(-(t305 * t305) + 1.0));
				t294_tmp = (-(t260_tmp * t304 * t353) + t248_tmp * t353 * t246) +
					t294_tmp * t137_tmp;
				t278 = (-(t92_tmp * t339) + t31_tmp * t341) + t121_tmp * t241;
				robot_ref_tcp_vec[5] =
					atan2(-t29 * ((t40 * t294_tmp + t340 * t238) + t258 * t232) -
						t42 * ((-t180 * t238 + t195 * t232) + t316 * t294_tmp),
						t29 * ((-t40 * t251 + t258 * t261) + t340 * t278) -
						t42 * ((-t195 * t261 + t180 * t278) + t316 * t251));
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t37_tmp;
				t278 = t37_tmp * (1.0 / (t62_tmp + t65_tmp));
				t305 = t68_tmp * t307;
				t342 = t11_tmp * t307;
				t340 = t95_tmp - t33_tmp * t216_tmp;
				t276 = t13_tmp * t340 + t35_tmp * t329_tmp;
				t340 = -t13_tmp * t329_tmp + t35_tmp * t340;
				t217 = t208_tmp * t146_tmp - t13_tmp * t299;
				robot_ref_tcp_vec[0] =
					(((((-t307 * (((kinePrt.refFrame_to_workspace[0] * t21_tmp * t22_tmp + kinePrt.refFrame_to_workspace[1] * t21_tmp * t38_tmp) -
						t294_tmp * t63_tmp) -
						t294_tmp * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t278 * t328 + t305 * t276) - t342 * t355)) +
						kinePrt.endFrame_to_tcp[2] * ((t278 * t302 + t305 * t322) + t342 * t324)) +
						kinePrt.endFrame_to_tcp[0] * ((t278 * t217 + t305 * t340) - t342 * t357)) -
						t278 * t358) +
						t305 * t367) +
					t342 * t368;
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t21_tmp * t36_tmp;
				t278 = t260_tmp * t366;
				t305 = t193_tmp * t306;
				robot_ref_tcp_vec[1] =
					(((((-t366 * (((((((kinePrt.refFrame_to_workspace[0] * t143_tmp + kinePrt.refFrame_to_workspace[1] * t145_tmp) +
						kinePrt.refFrame_to_workspace[1] * t163_tmp) -
						kinePrt.refFrame_to_workspace[0] * t165_tmp) +
						kinePrt.refFrame_to_workspace[1] * t164_tmp) -
						kinePrt.refFrame_to_workspace[0] * t167_tmp) +
						t294_tmp * t63_tmp) +
						t294_tmp * t66_tmp) -
						kinePrt.endFrame_to_tcp[1] * ((t278 * t276 + t248_tmp * t355 * t366) + t305 * t328)) -
						kinePrt.endFrame_to_tcp[2] * ((-t248_tmp * t324 * t366 + t260_tmp * t322 * t366) +
							t193_tmp * t302 * t306)) -
						kinePrt.endFrame_to_tcp[0] * ((t278 * t340 + t248_tmp * t357 * t366) + t305 * t217)) +
						t248_tmp * t366 * t368) -
						t278 * t367) +
					t305 * t358;
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t20_tmp * t21_tmp;
				t278 = t250_tmp * t366;
				t305 = t140_tmp * t306;
				robot_ref_tcp_vec[2] =
					(((((-t366 * (((((((kinePrt.refFrame_to_workspace[0] * t138_tmp + kinePrt.refFrame_to_workspace[1] * t142_tmp) -
						kinePrt.refFrame_to_workspace[1] * t166_tmp) +
						kinePrt.refFrame_to_workspace[0] * t169_tmp) -
						kinePrt.refFrame_to_workspace[1] * t168_tmp) +
						kinePrt.refFrame_to_workspace[0] * t170_tmp) +
						t294_tmp * t63_tmp) +
						t294_tmp * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t278 * t276 + t259_tmp * t355 * t366) - t305 * t328)) -
						kinePrt.endFrame_to_tcp[2] * ((-t250_tmp * t322 * t366 + t259_tmp * t324 * t366) +
							t140_tmp * t302 * t306)) +
						kinePrt.endFrame_to_tcp[0] * ((t278 * t340 + t259_tmp * t357 * t366) - t305 * t217)) +
						t278 * t367) -
						t259_tmp * t366 * t368) +
					t305 * t358;
			}
			//
			inline void PTP_Tool_OrientationRz(ValType* q, ValType tool_rel, ValType* robot_ref_tcp_vec)
			{
				double t11_tmp;
				double t121_tmp;
				double t12_tmp;
				double t134_tmp;
				double t135_tmp;
				double t136_tmp;
				double t137_tmp;
				double t138_tmp;
				double t13_tmp;
				double t140_tmp;
				double t142_tmp;
				double t143_tmp;
				double t145_tmp;
				double t146_tmp;
				double t162;
				double t163_tmp;
				double t164_tmp;
				double t165_tmp;
				double t166_tmp;
				double t167_tmp;
				double t168_tmp;
				double t169_tmp;
				double t170_tmp;
				double t180;
				double t181_tmp;
				double t193_tmp;
				double t194_tmp;
				double t195;
				double t195_tmp;
				double t206_tmp;
				double t208_tmp;
				double t20_tmp;
				double t216_tmp;
				double t217;
				double t21_tmp;
				double t22_tmp;
				double t232;
				double t238;
				double t241;
				double t244;
				double t246;
				double t248_tmp;
				double t249;
				double t250_tmp;
				double t251;
				double t258;
				double t259_tmp;
				double t26;
				double t260_tmp;
				double t261;
				double t27;
				double t275;
				double t276;
				double t278;
				double t28;
				double t29;
				double t290;
				double t293;
				double t294;
				double t294_tmp;
				double t299;
				double t302;
				double t304;
				double t305;
				double t306;
				double t307;
				double t316;
				double t31_tmp;
				double t322;
				double t324;
				double t328;
				double t329_tmp;
				double t339;
				double t33_tmp;
				double t340;
				double t341;
				double t342;
				double t34_tmp;
				double t353;
				double t355;
				double t357;
				double t358;
				double t35_tmp;
				double t366;
				double t367;
				double t368;
				double t36_tmp;
				double t37_tmp;
				double t385;
				double t38_tmp;
				double t39;
				double t40;
				double t41;
				double t42;
				double t62_tmp;
				double t63_tmp;
				double t65_tmp;
				double t66_tmp;
				double t68_tmp;
				double t69;
				double t74;
				double t76_tmp;
				double t8_tmp;
				double t93_tmp;
				double t96_tmp;
				double t9_tmp;
				t8_tmp = cos(q[0]);
				t9_tmp = cos(q[1]);
				t217 = cos(q[2]);
				t11_tmp = cos(q[3]);
				t12_tmp = cos(q[4]);
				t13_tmp = cos(q[5]);
				t20_tmp = cos(kinePrt.refFrame_to_workspace[3]);
				t21_tmp = cos(kinePrt.refFrame_to_workspace[4]);
				t22_tmp = cos(kinePrt.refFrame_to_workspace[5]);
				t26 = cos(kinePrt.endFrame_to_tcp[3]);
				t27 = cos(kinePrt.endFrame_to_tcp[4]);
				t28 = cos(kinePrt.endFrame_to_tcp[5]);
				t29 = cos(tool_rel);
				t357 = sin(q[0]);
				t31_tmp = sin(q[1]);
				t340 = sin(q[2]);
				t33_tmp = sin(q[3]);
				t34_tmp = sin(q[4]);
				t35_tmp = sin(q[5]);
				t36_tmp = sin(kinePrt.refFrame_to_workspace[3]);
				t37_tmp = sin(kinePrt.refFrame_to_workspace[4]);
				t38_tmp = sin(kinePrt.refFrame_to_workspace[5]);
				t39 = sin(kinePrt.endFrame_to_tcp[3]);
				t40 = sin(kinePrt.endFrame_to_tcp[4]);
				t41 = sin(kinePrt.endFrame_to_tcp[5]);
				t42 = sin(tool_rel);
				t276 = t20_tmp * t20_tmp;
				t62_tmp = t21_tmp * t21_tmp;
				t63_tmp = t22_tmp * t22_tmp;
				t290 = t36_tmp * t36_tmp;
				t65_tmp = t37_tmp * t37_tmp;
				t66_tmp = t38_tmp * t38_tmp;
				t68_tmp = t9_tmp * t217;
				t69 = t26 * t28;
				t341 = t9_tmp * t340;
				t339 = t217 * t31_tmp;
				t74 = t26 * t41;
				t76_tmp = t31_tmp * t340;
				t93_tmp = t8_tmp * t33_tmp;
				t96_tmp = t11_tmp * t357;
				t261 = t276 * t62_tmp;
				t121_tmp = t62_tmp * t63_tmp;
				t342 = t276 * t65_tmp;
				t305 = t62_tmp * t290;
				t258 = t63_tmp * t65_tmp;
				t316 = t290 * t65_tmp;
				t217 = t20_tmp * t22_tmp;
				t138_tmp = t217 * t37_tmp;
				t142_tmp = t20_tmp * t37_tmp * t38_tmp;
				t340 = t22_tmp * t36_tmp;
				t143_tmp = t340 * t37_tmp;
				t145_tmp = t36_tmp * t37_tmp * t38_tmp;
				t146_tmp = t341 + t339;
				t180 = t69 + t39 * t40 * t41;
				t134_tmp = t8_tmp * t68_tmp;
				t135_tmp = t8_tmp * t341;
				t136_tmp = t8_tmp * t339;
				t137_tmp = t357 * t68_tmp;
				t140_tmp = t357 * t341;
				t367 = t357 * t339;
				t162 = 1.0 / (t62_tmp + t65_tmp);
				t163_tmp = t217 * t62_tmp;
				t164_tmp = t217 * t65_tmp;
				t217 = t20_tmp * t38_tmp;
				t165_tmp = t217 * t62_tmp;
				t166_tmp = t340 * t62_tmp;
				t167_tmp = t217 * t65_tmp;
				t168_tmp = t340 * t65_tmp;
				t217 = t36_tmp * t38_tmp;
				t169_tmp = t217 * t62_tmp;
				t170_tmp = t217 * t65_tmp;
				t181_tmp = t68_tmp + -t76_tmp;
				t193_tmp = t135_tmp + t136_tmp;
				t194_tmp = t140_tmp + t367;
				t195_tmp = t28 * t39;
				t195 = t74 + -(t195_tmp * t40);
				t206_tmp = t11_tmp * t34_tmp;
				t208_tmp = t33_tmp * t35_tmp;
				t248_tmp = (t145_tmp + t163_tmp) + t164_tmp;
				t250_tmp = (t138_tmp + t169_tmp) + t170_tmp;
				t259_tmp = (-t142_tmp + t166_tmp) + t168_tmp;
				t260_tmp = (-t143_tmp + t165_tmp) + t167_tmp;
				t216_tmp = t134_tmp + t8_tmp * -t76_tmp;
				t385 = t357 * t76_tmp;
				t217 = t385 + -t137_tmp;
				t238 = t206_tmp * t146_tmp;
				t293 = 1.0 / (((t261 + t342) + t305) + t316);
				t278 = t62_tmp * t66_tmp;
				t294_tmp = t65_tmp * t66_tmp;
				t294 = 1.0 / (((t121_tmp + t278) + t258) + t294_tmp);
				t232 = t12_tmp * t181_tmp;
				t241 = t137_tmp + t357 * -t76_tmp;
				t244 = t12_tmp * t193_tmp;
				t246 = t12_tmp * t194_tmp;
				t340 = t11_tmp * t12_tmp;
				t275 = t340 * t146_tmp + t34_tmp * t181_tmp;
				t306 = 1.0 / (((t261 + t342) + t305) + t316);
				t307 = 1.0 / (((t121_tmp + t278) + t258) + t294_tmp);
				t353 = 1.0 / (((((((t66_tmp * t261 + t63_tmp * t342) + t290 * t121_tmp) +
					t66_tmp * t342) +
					t66_tmp * t305) +
					t290 * t258) +
					t66_tmp * t316) +
					t63_tmp * t261);
				t249 = t93_tmp + t11_tmp * t217;
				t276 = t357 * t33_tmp;
				t251 = t276 + t11_tmp * t216_tmp;
				t299 = t34_tmp * t181_tmp + t340 * t146_tmp;
				t302 = t232 + -t238;
				t366 = 1.0 / (((((((t63_tmp * t261 + t66_tmp * t261) + t63_tmp * t342) +
					t290 * t121_tmp) +
					t66_tmp * t342) +
					t66_tmp * t305) +
					t290 * t258) +
					t66_tmp * t316);
				t340 = t8_tmp * t11_tmp;
				t258 = t340 + -(t33_tmp * t217);
				t261 = t96_tmp + -(t33_tmp * t216_tmp);
				t276 += t11_tmp * t216_tmp;
				t278 = t340 + t33_tmp * t241;
				t340 = t13_tmp * t33_tmp;
				t316 = t340 * t146_tmp + t35_tmp * t275;
				t290 = t34_tmp * t276;
				t304 = t12_tmp * t193_tmp + t34_tmp * t251;
				t305 = t34_tmp * t194_tmp + t12_tmp * t249;
				t294_tmp = t93_tmp - t11_tmp * t241;
				t324 = t246 + -t34_tmp * t294_tmp;
				t328 = t340 * t146_tmp + t35_tmp * t299;
				t358 = (((((t31_tmp * 0.34 + t341 / 25.0) + t339 / 25.0) + t76_tmp * 0.338) +
					-(t68_tmp * 0.338)) +
					-(t232 * 0.0865)) +
					t238 * 0.0865;
				t368 = ((((((t357 * 0.03 + t9_tmp * t357 * 0.34) + t137_tmp / 25.0) +
					-(t385 / 25.0)) +
					t140_tmp * 0.338) +
					t367 * 0.338) +
					t246 * 0.0865) +
					t34_tmp * t294_tmp * -0.0865;
				t322 = t244 + t290;
				t329_tmp = t34_tmp * t193_tmp - t12_tmp * t276;
				t217 = t34_tmp * t193_tmp - t12_tmp * t251;
				t341 = t35_tmp * t261 + -t13_tmp * t217;
				t342 = t13_tmp * t261 + t35_tmp * t217;
				t217 = t34_tmp * t194_tmp + t12_tmp * t294_tmp;
				t355 = t13_tmp * t278 + -t35_tmp * t217;
				t357 = t35_tmp * t278 + t13_tmp * t217;
				t339 = t35_tmp * t258 + t13_tmp * t305;
				t68_tmp = t21_tmp * t22_tmp;
				t31_tmp = t68_tmp * t294;
				t367 = ((((((t8_tmp * 0.03 + t8_tmp * t9_tmp * 0.34) + t134_tmp / 25.0) +
					-(t8_tmp * t76_tmp / 25.0)) +
					t135_tmp * 0.338) +
					t136_tmp * 0.338) +
					t244 * 0.0865) +
					t290 * 0.0865;
				t137_tmp = t206_tmp * t146_tmp - t12_tmp * t181_tmp;
				t246 = t12_tmp * t194_tmp - t34_tmp * t249;
				t11_tmp = t21_tmp * t38_tmp;
				t93_tmp = t11_tmp * t294;
				t251 = (-t37_tmp * t162 * t137_tmp + t31_tmp * t304) + t93_tmp * t246;
				t140_tmp = t20_tmp * t21_tmp;
				t217 = t140_tmp * t293;
				t385 = (t217 * t137_tmp + t250_tmp * t304 * t353) + -t259_tmp * t353 * t246;
				t340 = t13_tmp * t258 + -(t35_tmp * t305);
				t241 = t208_tmp * t146_tmp - t13_tmp * t275;
				t290 = (-t20_tmp * t21_tmp * t293 * t241 + t250_tmp * t341 * t353) +
					t259_tmp * t339 * t353;
				t193_tmp = t21_tmp * t36_tmp;
				t294_tmp = t193_tmp * t293;
				t238 = (t294_tmp * t241 + t248_tmp * t339 * t353) + t260_tmp * t341 * t353;
				t232 = (t294_tmp * t316 + t248_tmp * t340 * t353) + t260_tmp * t342 * t353;
				t278 = (-(t217 * t316) + t250_tmp * t342 * t353) + t259_tmp * t340 * t353;
				t121_tmp = t37_tmp * t162;
				t261 = (t121_tmp * t316 + t31_tmp * t342) + -(t93_tmp * t340);
				t340 = t27 * t28;
				t258 = t27 * t41;
				t217 = (-(t40 * t385) + t340 * t290) + t258 * t278;
				t316 = t27 * t39;
				t276 = (t316 * t385 + t180 * t278) + -(t195 * t290);
				t342 = t42 * t276;
				t305 = t29 * t217 + t342;
				robot_ref_tcp_vec[3] =
					atan2(t29 * t276 - t42 * t217, (-t278 * (t195_tmp - t40 * t74) +
						t290 * (t39 * t41 + t40 * t69)) +
						t26 * t27 * t385);
				robot_ref_tcp_vec[4] =
					atan2(-t29 * t217 - t342, sqrt(-(t305 * t305) + 1.0));
				t294_tmp = (-(t260_tmp * t304 * t353) + t248_tmp * t353 * t246) +
					t294_tmp * t137_tmp;
				t278 = (-(t93_tmp * t339) + t31_tmp * t341) + t121_tmp * t241;
				robot_ref_tcp_vec[5] =
					atan2(-t29 * ((t40 * t294_tmp + t340 * t238) + t258 * t232) +
						t42 * ((-t180 * t232 + t195 * t238) + t316 * t294_tmp),
						t29 * ((-t40 * t251 + t258 * t261) + t340 * t278) +
						t42 * ((t180 * t261 - t195 * t278) + t316 * t251));
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t37_tmp;
				t278 = t37_tmp * (1.0 / (t62_tmp + t65_tmp));
				t305 = t68_tmp * t307;
				t342 = t11_tmp * t307;
				t340 = t96_tmp - t33_tmp * t216_tmp;
				t276 = t13_tmp * t340 + t35_tmp * t329_tmp;
				t340 = -t13_tmp * t329_tmp + t35_tmp * t340;
				t217 = t208_tmp * t146_tmp - t13_tmp * t299;
				robot_ref_tcp_vec[0] =
					(((((-t307 * (((kinePrt.refFrame_to_workspace[0] * t21_tmp * t22_tmp + kinePrt.refFrame_to_workspace[1] * t21_tmp * t38_tmp) -
						t294_tmp * t63_tmp) -
						t294_tmp * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t278 * t328 + t305 * t276) - t342 * t355)) +
						kinePrt.endFrame_to_tcp[2] * ((t278 * t302 + t305 * t322) + t342 * t324)) +
						kinePrt.endFrame_to_tcp[0] * ((t278 * t217 + t305 * t340) - t342 * t357)) -
						t278 * t358) +
						t305 * t367) +
					t342 * t368;
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t21_tmp * t36_tmp;
				t278 = t260_tmp * t366;
				t305 = t193_tmp * t306;
				robot_ref_tcp_vec[1] =
					(((((-t366 * (((((((kinePrt.refFrame_to_workspace[0] * t143_tmp + kinePrt.refFrame_to_workspace[1] * t145_tmp) +
						kinePrt.refFrame_to_workspace[1] * t163_tmp) -
						kinePrt.refFrame_to_workspace[0] * t165_tmp) +
						kinePrt.refFrame_to_workspace[1] * t164_tmp) -
						kinePrt.refFrame_to_workspace[0] * t167_tmp) +
						t294_tmp * t63_tmp) +
						t294_tmp * t66_tmp) -
						kinePrt.endFrame_to_tcp[1] * ((t278 * t276 + t248_tmp * t355 * t366) + t305 * t328)) -
						kinePrt.endFrame_to_tcp[2] * ((-t248_tmp * t324 * t366 + t260_tmp * t322 * t366) +
							t193_tmp * t302 * t306)) -
						kinePrt.endFrame_to_tcp[0] * ((t278 * t340 + t248_tmp * t357 * t366) + t305 * t217)) +
						t248_tmp * t366 * t368) -
						t278 * t367) +
					t305 * t358;
				t294_tmp = kinePrt.refFrame_to_workspace[2] * t20_tmp * t21_tmp;
				t278 = t250_tmp * t366;
				t305 = t140_tmp * t306;
				robot_ref_tcp_vec[2] =
					(((((-t366 * (((((((kinePrt.refFrame_to_workspace[0] * t138_tmp + kinePrt.refFrame_to_workspace[1] * t142_tmp) -
						kinePrt.refFrame_to_workspace[1] * t166_tmp) +
						kinePrt.refFrame_to_workspace[0] * t169_tmp) -
						kinePrt.refFrame_to_workspace[1] * t168_tmp) +
						kinePrt.refFrame_to_workspace[0] * t170_tmp) +
						t294_tmp * t63_tmp) +
						t294_tmp * t66_tmp) +
						kinePrt.endFrame_to_tcp[1] * ((t278 * t276 + t259_tmp * t355 * t366) - t305 * t328)) -
						kinePrt.endFrame_to_tcp[2] * ((-t250_tmp * t322 * t366 + t259_tmp * t324 * t366) +
							t140_tmp * t302 * t306)) +
						kinePrt.endFrame_to_tcp[0] * ((t278 * t340 + t259_tmp * t357 * t366) - t305 * t217)) +
						t278 * t367) -
						t259_tmp * t366 * t368) +
					t305 * t358;
			}
			//
			inline unsigned short isOverTraveled(ValType* q, unsigned short unit=UNIT_RAD) {
				/*
				* 結構: joint_limit[2][6]:
				*	joint_limit[0]: NOT 6軸正極限
				*	joint_limit[1]: POT 6軸負極限
				*/
				unsigned short err = 0;
				if (unit == UNIT_RAD) {
					if (q[0] <= kinePrt.joint_limit[0][0])
						err |= static_cast<unsigned short>(RobotError::J1NOT);
					if (q[0] >= kinePrt.joint_limit[1][0])
						err |= static_cast<unsigned short>(RobotError::J1POT);
					if (q[1] <= kinePrt.joint_limit[0][1])
						err |= static_cast<unsigned short>(RobotError::J2NOT);
					if (q[1] >= kinePrt.joint_limit[1][1])
						err |= static_cast<unsigned short>(RobotError::J2POT);
					if (q[2] <= kinePrt.joint_limit[0][2])
						err |= static_cast<unsigned short>(RobotError::J3NOT);
					if (q[2] >= kinePrt.joint_limit[1][2])
						err |= static_cast<unsigned short>(RobotError::J3POT);
					if (q[3] <= kinePrt.joint_limit[0][3])
						err |= static_cast<unsigned short>(RobotError::J4NOT);
					if (q[3] >= kinePrt.joint_limit[1][3])
						err |= static_cast<unsigned short>(RobotError::J4POT);
					if (q[4] <= kinePrt.joint_limit[0][4])
						err |= static_cast<unsigned short>(RobotError::J5NOT);
					if (q[4] >= kinePrt.joint_limit[1][4])
						err |= static_cast<unsigned short>(RobotError::J5POT);
					if (q[5] <= kinePrt.joint_limit[0][5])
						err |= static_cast<unsigned short>(RobotError::J6NOT);
					if (q[5] >= kinePrt.joint_limit[1][5])
						err |= static_cast<unsigned short>(RobotError::J6POT);
				}
				return err;
			}
			//
			inline unsigned short isSingular(ValType* q) {
				unsigned short err = 0;
				if (abs(q[4]) <= kinePrt.singular_threshold.wrist) { 
					// 判斷 q5 角度是否在 +- WRIST_THRESHOLD 以內
					err |= static_cast<unsigned short>(RobotError::WristSingular);
				}
				if (abs(kinePrt.DH.a[3] * sin(q[2]) - kinePrt.DH.d[3] * cos(q[2])) <= kinePrt.singular_threshold.elbow) 
					// 判斷手腕點，距離第二軸座標系之 x 軸線的直線距離是否在 ELBOW_THRESHOLD 以內
					err |= static_cast<unsigned short>(RobotError::ElbowSingular);
				if((kinePrt.DH.a[1] + kinePrt.DH.a[3]*cos(q[1] + q[2]) + kinePrt.DH.d[3]*sin(q[1] + q[2]) + kinePrt.DH.a[2]*cos(q[1])) <= kinePrt.singular_threshold.shoulder){ 
					// 判斷手腕點，距離 robot 座標系之座標點 (X0, Y0) = (0.0, 0.0) 之直線距離，是否在 SHOULDER_THRESHOLD 以內。(i.e. 距離第一軸馬達的軸線距離)
					err |= static_cast<unsigned short>(RobotError::ShoulderSingular);
				}
				return err;
			}
#pragma endregion
#pragma region utility
			inline void JointToCount(ValType* q, ValType* pulse) {

				pulse[0] = ((q[0] - M_PI_2) * (80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[0];
				pulse[1] = ((q[1] - M_PI_2) * (100.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[1];
				pulse[2] = (q[2] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[2];
				pulse[3] = (q[3] * (-81.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[3];
				pulse[4] = (q[4] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[4];
				pulse[5] = ((q[5] * -50.0) - q[4]) * kinePrt.PPU + kinePrt.encoder_zero_pulse[5];
			}
			//
			inline void CountToJoint(ValType* pulse, ValType* q) {
				q[0] = M_PI_2 + (pulse[0] - kinePrt.encoder_zero_pulse[0]) / 80.0 / kinePrt.PPU;
				q[1] = M_PI_2 + (pulse[1] - kinePrt.encoder_zero_pulse[1]) / 100.0 / kinePrt.PPU;
				q[2] = (pulse[2] - kinePrt.encoder_zero_pulse[2]) / -80.0 / kinePrt.PPU;
				q[3] = (pulse[3] - kinePrt.encoder_zero_pulse[3]) / -81.0 / kinePrt.PPU;
				q[4] = (pulse[4] - kinePrt.encoder_zero_pulse[4]) / -80.0 / kinePrt.PPU;
				q[5] = ((pulse[5] - kinePrt.encoder_zero_pulse[5]) / kinePrt.PPU + q[4]) / -50.0;
			}
			//
			inline void JointToMotor(ValType* q, ValType* m) {
				m[0] = q[0] * 80.0;
				m[1] = q[1] * 100.0;
				m[2] = q[2] * -80.0;
				m[3] = q[3] * -81.0;
				m[4] = q[4] * -80;
				m[5] = (q[5] * -50) - q[4];
			}
			inline void Joint6ToMotor6(ValType* q, ValType* m) {
				m[5] = (q[5] * -50) - q[4];
			}
			//
			inline void MotorToJoint(ValType* m, ValType* q) {
				q[0] = m[0] / 80.0;
				q[1] = m[1] / 100.0;
				q[2] = m[2] / -80.0;
				q[3] = m[3] / -81.0;
				q[4] = m[4] / -80.0;
				q[5] = (m[5] + q[4]) / -50.0;
			}
			//
			inline void JointToCount(std::array<ValType, 6>& target) {
				target[0] = ((target[0] - M_PI_2) * (80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[0];
				target[1] = ((target[1] - M_PI_2) * (100.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[1];
				target[2] = (target[2] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[2];
				target[3] = (target[3] * (-81.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[3];
				target[5] = ((target[5] * -50.0) - target[4]) * kinePrt.PPU + kinePrt.encoder_zero_pulse[5];
				target[4] = (target[4] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[4];
			}
			//
			inline void JointToCount(ValType* target) {
				target[0] = ((target[0] - M_PI_2) * (80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[0];
				target[1] = ((target[1] - M_PI_2) * (100.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[1];
				target[2] = (target[2] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[2];
				target[3] = (target[3] * (-81.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[3];
				target[5] = ((target[5] * -50.0) - target[4]) * kinePrt.PPU + kinePrt.encoder_zero_pulse[5];
				target[4] = (target[4] * (-80.0 * kinePrt.PPU)) + kinePrt.encoder_zero_pulse[4];
			}
			//
			inline void CountToJoint(std::array<ValType, 6>& target) {
				target[0] = M_PI_2 + (target[0] - kinePrt.encoder_zero_pulse[0]) / 80.0 / kinePrt.PPU;
				target[1] = M_PI_2 + (target[1] - kinePrt.encoder_zero_pulse[1]) / 100.0 / kinePrt.PPU;
				target[2] = (target[2] - kinePrt.encoder_zero_pulse[2]) / -80.0 / kinePrt.PPU;
				target[3] = (target[3] - kinePrt.encoder_zero_pulse[3]) / -81.0 / kinePrt.PPU;
				target[4] = (target[4] - kinePrt.encoder_zero_pulse[4]) / -80.0 / kinePrt.PPU;
				target[5] = ((target[5] - kinePrt.encoder_zero_pulse[5]) / kinePrt.PPU + target[4]) / -50.0;
			}
			//
			inline void CountToJoint(ValType* target) {
				target[0] = M_PI_2 + (target[0] - kinePrt.encoder_zero_pulse[0]) / 80.0 / kinePrt.PPU;
				target[1] = M_PI_2 + (target[1] - kinePrt.encoder_zero_pulse[1]) / 100.0 / kinePrt.PPU;
				target[2] = (target[2] - kinePrt.encoder_zero_pulse[2]) / -80.0 / kinePrt.PPU;
				target[3] = (target[3] - kinePrt.encoder_zero_pulse[3]) / -81.0 / kinePrt.PPU;
				target[4] = (target[4] - kinePrt.encoder_zero_pulse[4]) / -80.0 / kinePrt.PPU;
				target[5] = ((target[5] - kinePrt.encoder_zero_pulse[5]) / kinePrt.PPU + target[4]) / -50.0;
			}
			//
			inline void CountToMotor(std::array<ValType, 6>& target) {
				target[0] = M_PI_2 * 80.0 + (target[0] - kinePrt.encoder_zero_pulse[0]) / kinePrt.PPU;
				target[1] = M_PI_2 * 100.0 + (target[1] - kinePrt.encoder_zero_pulse[1]) / kinePrt.PPU;
				target[2] = (target[2] - kinePrt.encoder_zero_pulse[2]) / -kinePrt.PPU;
				target[3] = (target[3] - kinePrt.encoder_zero_pulse[3]) / -kinePrt.PPU;
				target[4] = (target[4] - kinePrt.encoder_zero_pulse[4]) / -kinePrt.PPU;
				target[5] = ((target[5] - kinePrt.encoder_zero_pulse[5]) / -kinePrt.PPU - target[4]);
			}
			//
			inline void JointToMotor(std::array<ValType, 6>& target) {
				target[0] = target[0] * 80.0;
				target[1] = target[1] * 100.0;
				target[2] = target[2] * -80.0;
				target[3] = target[3] * -81.0;
				target[5] = (target[5] * -50) - target[4];
				target[4] = target[4] * -80;

			}
			inline void Joint6ToMotor6(std::array<ValType, 6>& target) {
				target[5] = (target[5] * -50) - target[4];
			}
			//
			inline void MotorToJoint(std::array<ValType, 6>& target) {
				target[0] = target[0] / 80.0;
				target[1] = target[1] / 100.0;
				target[2] = target[2] / 80.0;
				target[3] = target[3] / 81.0;
				target[4] = target[4] / 80.0;
				target[5] = (target[5] + target[4]) / 50.0;
			}
#pragma endregion utility
	};
};


#pragma region Example codes:
/* Robotics Inverse Kinematics:
* ==============================================================================================================================
 	ICAL::Robot<double> RT605(ICALab_Robot::Hiwin_RT605);  // Generate Robot object
	std::array<double, 6> pos = { 0, 0.368, 0.2935, M_PI, 0, M_PI_2 }; // Generate Cartesian position vector
	IK_Sol<double> q_sol; // Declare a set of q-solutions.
	RT605.RT605_IK_Sol8(pos, q_sol, UNIT_DEGREE); // Execute IK.
	double q[6];
	q_sol.ReadSol(Robot_Postures::MarkUp_ElbowUp_WristFronted, q); // Specify (or Designate) the robot posture.
	// Print IK result:
	for (int i = 0; i < 6; ++i)
		std::cout << q[i] << std::endl;
	// Print the specified IK_SolutionState of q-solution:
	std::cout << "\n" << static_cast<char>(q_sol.singularStates.at(Robot_Postures::MarkUp_ElbowUp_WristFronted)) << std::endl;
==============================================================================================================================
*/
#pragma endregion