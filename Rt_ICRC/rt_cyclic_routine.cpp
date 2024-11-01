/*
  - Create: 2022/07/07, b.r.tseng
  - Edit: 2023/05/28, b.r.tseng
*/
//////////////////////////////////////////////////////////////////
#include"rt_cyclic_routine.h"

//
void RTFCNDCL RtMotionRoutine_Position(void* _context) {
	// 連結機器手臂控制器之全域物件：
	RT_Robot_Controller* rc = (RT_Robot_Controller*)(_context);
	// ---------------------------------------------
	// Only Position Control Mode:
	// ---------------------------------------------
	for (unsigned short i = 0; i < 6; ++i)
		rc->xc_update.at(i) = rc->Xr_buf[rc->xr_index][i];
	rc->xr_index += 1;

	// 關節姿態解的選擇：
	if (rc->feedback[0].q[4] >= 0.0)
		rc->rt605.RT605_IK_Config1(rc->xc_update.data(), rc->qc_update.data());
	else
		rc->rt605.RT605_IK_Config2(rc->xc_update.data(), rc->qc_update.data());

	rc->SetRobotJointTargetPosition(rc->qc_update.data());
	if (rc->xr_index == rc->Xr_length) {
		rc->TerminateCspTask();
		rc->xr_index = rc->Xr_length - 1;
	}
}
//
DWORD RTFCNDCL DataLoggerThread(void* _context) {
	// 連結機器手臂控制器之全域物件：
	RT_Robot_Controller* rc = static_cast<RT_Robot_Controller*>(_context);
	//while (rc->logger.isStarted()) { 
	//	RtWaitForSingleObject(rc->hEvent_logger, INFINITE);
	//}
	double xc[6];
	double qc[6];
	while (1) {
		memcpy(xc, rc->Xr_buf[0], 6 * sizeof(double));
		RtPrintf("%i,%i,%i,%i,%i,%i\n", (int)(rc->xc_update[0] * 1000.0),
			(int)(xc[1] * 1000.0),
			(int)(xc[2] * 1000.0),
			(int)(xc[3] * 1000.0),
			(int)(xc[4] * 1000.0),
			(int)(xc[5] * 1000.0));

		//RtPrintf("\n%i,%i,%i,%i,%i,%i\n", (int)(rad2deg(qc[0])), (int)(rad2deg(qc[1])), (int)(rad2deg(qc[2])), \
		//	(int)(rad2deg(qc[3]), (int)(rad2deg(qc[4])), (int)(rad2deg(qc[5]))));
		RtPrintf("\n%i, %i, %i, %i, %i, %i\n", (int)rad2deg(qc[0]), (int)rad2deg(qc[1]), (int)rad2deg(qc[2]), (int)rad2deg(qc[3]), (int)rad2deg(qc[4]), (int)rad2deg(qc[5]));
		RtSleep(500);

	}
	return 0;
}
//
void RTFCNDCL MotionFeedbackRoutine(void* _context) {	// 0.5 ms

	static bool first_time{ true };
	if (first_time) {
		RtPrintf("Start motion feedback routine. (Thread ID: %#04x)\n", GetCurrentThreadId());
		first_time = false;
	}
	RT_Robot_Controller* rc = static_cast<RT_Robot_Controller*>(_context);
	// Update Motion Feedback:
	rc->UpdateFeedback();
	//// Log:
	if (rc->logger_on == true) {
		// export:
		fprintf(rc->logger_fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			rc->feedback[0].time,
			rc->feedback[0].Xr[0], rc->feedback[0].Xr[1], rc->feedback[0].Xr[2], rc->feedback[0].Xr[3], rc->feedback[0].Xr[5], rc->feedback[0].Xr[6],
			rc->feedback[0].qc[0], rc->feedback[0].qc[1], rc->feedback[0].qc[2], rc->feedback[0].qc[3], rc->feedback[0].qc[4], rc->feedback[0].qc[5],
			rc->feedback[0].q[0], rc->feedback[0].q[1], rc->feedback[0].q[2], rc->feedback[0].q[3], rc->feedback[0].q[4], rc->feedback[0].q[5],
			rc->feedback[0].tor[0], rc->feedback[0].tor[1], rc->feedback[0].tor[2], rc->feedback[0].tor[3], rc->feedback[0].tor[4], rc->feedback[0].tor[5]);
			//rc->feedback[0].fc_command[0], rc->feedback[0].fc_command[1], rc->feedback[0].fc_command[2], rc->feedback[0].fc_command[3], rc->feedback[0].fc_command[4], rc->feedback[0].fc_command[5], // force controller: force command
			//rc->feedback[0].fts.raw[0], rc->feedback[0].fts.raw[1], rc->feedback[0].fts.raw[2], rc->feedback[0].fts.raw[3], rc->feedback[0].fts.raw[4], rc->feedback[0].fts.raw[5],
			//rc->feedback[0].fts.tcp[0], rc->feedback[0].fts.tcp[1], rc->feedback[0].fts.tcp[2], rc->feedback[0].fts.tcp[3], rc->feedback[0].fts.tcp[4], rc->feedback[0].fts.tcp[5],
			//rc->feedback[0].fc_output[0], rc->feedback[0].fc_output[1], rc->feedback[0].fc_output[2], rc->feedback[0].fc_output[3], rc->feedback[0].fc_output[4], rc->feedback[0].fc_output[5],
			//rc->feedback[0].fc_gain[0], rc->feedback[0].fc_gain[1], rc->feedback[0].fc_gain[2], rc->feedback[0].fc_gain[3], rc->feedback[0].fc_gain[4], rc->feedback[0].fc_gain[5]);
	}
	memcpy(rc->pShared_feedback, &rc->feedback[0], sizeof(rc->feedback[0]));
	// Safety Detect:
}
//
void RTFCNDCL SafetyDetectRoutine(void* _context) { // 0.5 ms
	static bool first_time{ true };
	if (first_time) {
		RtPrintf("Start safety-detect routine. (Thread ID: %#04x)\n", GetCurrentThreadId());
		first_time = false;
	}
	RT_Robot_Controller* rc = static_cast<RT_Robot_Controller*>(_context);

	// update controller system state:
	rc->system_state->GetInformation(RcSystemStateAction::update, *(rc->system_state));
	// 檢察關節位置是否安全:

	/*
	unsigned short safety_state = rc->CheckRobotError();
	if (safety_state != 0 && !rc->GetRobotError()) {
		//rc->SetRobotError();
		//for (int i = 0; i < 6; ++i) {
		//	QuickStopServo(i);
		//}
		RtPrintf("%#04x\n", safety_state);
	}

	*/
}
//
