#pragma once
#include<tchar.h>
#include<string.h>
#include<stdio.h>
#include "ksapi.h"
#include "ksmotion.h"

#include "../libs/shared.hpp"
#include "../libs/icalab/RT605/rt605Kinematics.h"
//#include "../libs/icalab/RT605/rt605Utility.h"


#include"../libs/icalab/rc/ForceControl/rc_force_control.h"
#include"../libs/icalab/dsp/filter.h"
#include"../libs/icalab/rc/peripheral/peripheral.h"
#include"system_ethercat_setting.h"
#include"rt_cyclic_routine.h"


#include "MechanismAnalyzer.h"
#include "rt_cyclic_routine.h"
#include "SinusoidalTest.h"

#pragma region REDEFINE
#define ETHERCAT_CYCLE_ms cycle500
/*以下之 config 之後要改用讀取外部的 config.ini 檔*/
#define ETHERCAT_SLAVE_COUNT 15
#define ETHERCAT_IO_COUNT 9
#define ETHERCAT_SERVO_COUNT 6
#pragma endregion REDEFINE

#define MOTION_CYCLE 1.0  // 1.0 ms (unit: ms)
#define MOTION_FEEDBACK_CYCLE 0.5 // 0.5ms (unit: ms)
#define SAFETY_DETECT_CYCLE 0.4 // 0.4 ms
#define ETHERCAT_CYCLE CycleTime::cycle500
#define FORCE_CONTROL_CYCLE 5000

#define JOINT_JOG_MODE false
#define CARTESIAN_JOG_MODE true



struct JogProfileSetting {
	double JointAcceleration_time; // unit: second
	double CartesianAcceleration_time; // unit: second
	double JointSpeed; // unit: rad/second
	double CartesianSpeed; // unit: mm/second
	double MaxJointSpeed; // unit: rad/second
	double MaxCartesianSpeed; // unit: mm/second
};

enum class LogMODE {
	None,
	sinusoidal,
	sweep,
	sweepALL
};

class RT_Robot_Controller
{
public:
	// Process attributes:
	char app_dir[MAX_PATH];
	char fopen_path[MAX_PATH];
	char save_file_path[MAX_PATH];
private:

	// ---------- Buffers: ----------
	unsigned long long Xr_buf_heap_capacity;
	// ---------- Periodic Timers: ----------
	// Motion Feedback:
	LARGE_INTEGER liPeriod_feedback;
	HANDLE hTimer_motion_feedback;
	// Safety Detect:
	LARGE_INTEGER liPeriod_safety;
	HANDLE hTimer_motion_safety;
	// 建立 logger 的執行續，並且處於待命中的狀態：
	HANDLE hEvent_logger;
	HANDLE hThread_logger;
	// Position Motion Control:
	LARGE_INTEGER liPeriod_motion;
	HANDLE hTimer_motion;

	//
	HANDLE hSystem_state;
	//
	HANDLE hShared_feedback;
	// 各種系統狀態：
	unsigned short m_error_state;

public:
	icalab::RT605<double> rt605;
	pRcSystemState system_state;
	McProfileSettings servo_profile[6];
	double OV_Speed; // Unit: deg/second

	// 運動狀態紀錄器
	FILE* logger_fs; // file stream
	bool  logger_on; // 紀錄器的 開啟/關閉 之狀態
	MotionFeedback feedback[2]; // 2組存放回授資訊的運動狀態紀錄器
	MotionFeedback* pShared_feedback; // 與 Window UI 共享的回授資訊之共享記憶體
	// 
	double** Xr_buf;
	unsigned long long Xr_length;

	unsigned long long xr_index;
	std::array<double, 6> xc_update; // Xc(k+1), unit: mm and rad
	std::array<double, 6> qc_update; // qc(k+1), unit: rad
	std::array<double, 6> mc_update; // mc(k+1), unit: pulse
	Robot_Postures robot_priorPosture;

	unsigned long long fc_start_point;
	unsigned long long fc_process_phrase[2];

	double pc_kff[3]; // position control-loop feedforward gain

	unsigned short ke_mode;

	icalab::MechanismAnalyzer mechanism_analyzer;
	icalab::SinusoidalTest sinusoidal_test;

	RT_Robot_Controller(void);
	~RT_Robot_Controller(void);
	int ConfigEtherCATslaveID(void);
	void GetRtssAbsolutePath(_TCHAR* rtss_full_path); //  rtss_full_path: 當前執行檔 *.rtss 的絕對路徑
	int Initialize(void);
	int stopEtherCAT(void);
	int destroyLink(void);
	void getIOState(void);
	void ImportRobotParameter(char* app_dir); // 當前存放執行檔 *.rtss 的目錄之絕對路徑
	int ServoON(void);
	int ServoOFF(void);
	int LoadTaskProgram(char* intp_file); // 任務專案的名稱，例如："exp1_20220906"
	int ReadHrssIntpFile(const char* _file); // HRSS intp 檔的絕對路徑
	int SetRobotTargetPosition(double* _pos); // 
	int SetRobotJointTargetPosition(double* _q); // _q: csp 模式下要送至6軸驅動器的角度命令 (角度單位為 rad)，例如：double qr[6] = {0.0, 0.0, 0.0, 0.0, -pi/2, 0.0};
	int MoveRobotAbsPTP(unsigned short _frame, double* _pos); // reserved
	int MoveRobotRelPTP(unsigned short _frame, double* _pos); // reserved
	//int MoveRobotAbsLINE(unsigned short _frame, double* _pos);
	//int MoveRobotRelLINE(unsigned short _frame, double* _pos);
	int MoveJointRel(int _index, double _q); // _index: 要移動的關節的index；_q: 要移動的相對角度 (角度單位為 rad)
	int MoveJointRel(double* _q); // _q: 6軸要移動的相對角度 (角度單位為 rad)
	int MoveJointAbs(double* _q); // _q: 6軸要移動的絕對角度 (角度單位為 rad)
	int MoveJointJog(int _index, double _vel); // _index: 要移動的關節的index；_vel: JOG的轉速 (單位為 rad/sec.)
	void SetOVSpeed(double _vel);
	int StopMove(void);
	int HaltMove(int);
	int HaltAll();
	int QuickStopRobot(void);
	int ResetRobotAxis(void);
	int ResetRobotAlarm(void);
	unsigned short GetRobotErrorState(void);
	int GoHome(double _goHome_speed); // _goHome_speed: 六軸回到原點的轉速(單位為 rad/sec.)
	int ConfigJogProfile(JogProfileSetting& _setting); // _setting: 設定 JOG模式的加速度等運動資訊。
													   /*  example:
																	JogProfileSetting setting;
																	setting.JointAcceleration_time = 500; // acc time = 500 ms
																	setting.JointSpeed = 0.35;			  // JOG SPEED = 0.35 rad/second
																	ConfigJogProfile(setting);
													*/
	void select_RobotPriorPosture(int _posture);
	void StartINTPtask(void);
	void TerminateCspTask(void);

	int readPID(PID *); //read PID controller parameters from rt605
	int writePID(PID); // write PID parameters to RT605
	

	/*
	* _clk_ms: interrupt period in ms; pRoutine: interrupt service routine
	*/
	void setFeedbackRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context));
	void setPositionMotionRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context));
	void setSafetyDetectRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context));

	void StartFeedbackRoutine(void);
	void StartSafetyDetectRoutine(void);
	void StartPositionMotionRoutine(void);
	//void StartPositionForceMotionRoutine(void);
	void UpdateFeedback(void);
	void LogOn(void);
	void LogOff(void);

	//void LogOn(const char*);
	void LogOn(const char*, LogMODE);
	//void ExportToolDynamicsIdentifyResult(void);
	void CancelRoutineTimer(void);
};

