#pragma once
/*
  - Create: 2022/07/07, b.r.tseng
  - Edit: 2023/05/28, b.r.tseng
*/
//////////////////////////////////////////////////////////////////
#ifndef __RT_ICRC_CYCLIC_ROUTINE_H__
#define __RT_ICRC_CYCLIC_ROUTINE_H__

#include"rt_robot_controller.h"
//#include"../libs/shared.hpp"

#include "shared.hpp"
#include "../libs/icalab/RT605/rt605Kinematics.h"
#include "MechanismAnalyzer.h"

// periodic timer functions: 		
void RTFCNDCL RtMotionRoutine_Position(void* _context); // 1.0 ms
//void RTFCNDCL RtMotionRoutine_HybridForcePosition(void* _context); // 1.0 ms
void RTFCNDCL MotionFeedbackRoutine(void* _context);	// 0.5 ms, motion log routine 
void RTFCNDCL SafetyDetectRoutine(void* _context);
DWORD RTFCNDCL DataLoggerThread(void* _context);

#endif 