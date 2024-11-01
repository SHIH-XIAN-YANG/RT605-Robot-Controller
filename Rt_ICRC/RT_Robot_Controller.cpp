#include "RT_Robot_Controller.h"


RT_Robot_Controller::RT_Robot_Controller(void){
	Xr_buf = nullptr;
	Xr_length = 0;
	Xr_buf_heap_capacity = 0;

	hShared_feedback = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(MotionFeedback), SHM_MOTIONFEEDBACK_NAME, (void**)&pShared_feedback);
	hSystem_state = RtCreateSharedMemory(SHM_MAP_WRITE, 0, sizeof(RcSystemState), SHM_SYSTEM_STATE, (void**)&system_state);
	logger_fs = nullptr;
}

RT_Robot_Controller::~RT_Robot_Controller(void){
	ServoOFF();

	RtCloseHandle(hThread_logger);
	RtCloseHandle(hTimer_motion);
	RtCloseHandle(hTimer_motion_feedback);
	RtCloseHandle(hTimer_motion_safety);

	RtCloseHandle(hShared_feedback);
	RtCloseHandle(hSystem_state);

	stopEtherCAT();
	destroyLink();
}

#pragma region ConnectionSetting

int RT_Robot_Controller::ConfigEtherCATslaveID(void){
	// 執行此程式前必須先執行 RobotController::GetRtssAbsolutePath(char* app_dir)，取得當前執行檔的根目錄
	int nRet = -1;
	char path[MAX_PATH]; 
	//strset(path, 0);
	char** slave_info_txt;
	FILE* fs = nullptr;
	errno_t err;
	unsigned short ethercat_slave_num = 0;
	strcpy(path, app_dir);
	strcat_s(path, "Parameters\\ethercat\\EtherCAT_slave_list.txt");
	err = fopen_s(&fs, path, "r");
	if (err != 0) {
		RtPrintf("Cannot open file: %s\n", path);
		return -1;
	}
	if (fs != nullptr) {
		for (int c = getc(fs); !feof(fs); c = getc(fs)) {
			if (c == '\n')
				++ethercat_slave_num;
		}
		fclose(fs);
		RtPrintf("EtherCAT slave number: %i\n", ethercat_slave_num);
		slave_info_txt = static_cast<char**>(malloc(sizeof(char*) * ethercat_slave_num));
		for (unsigned short i = 0; i < ethercat_slave_num; ++i)
			slave_info_txt[i] = static_cast<char*>(malloc(sizeof(char) * MAX_PATH));
		// Read "EtherCAT_slave_list.txt":
		fs = nullptr;
		err = fopen_s(&fs, path, "r");
		if (err != 0) {
			RtPrintf("Cannot open file: %s\n", path);
			return -1;
		}
		if (fs != nullptr) {
			for (unsigned short i = 0; i < ethercat_slave_num; ++i) {
				fscanf(fs, "%s\n", slave_info_txt[i]);
				RtPrintf("Slave-%i: %s\n", i, slave_info_txt[i]);
			}
			fclose(fs);
		}
		else {
			RtPrintf("Failed to open the file: %s.\n", path);
		}
		//
		fs = nullptr;
		for (unsigned short i = 0; i < ethercat_slave_num; ++i) {
			//strset(path, 0);
			strcpy(path, app_dir);
			strcat_s(path, "Parameters\\ethercat\\");
			strcat_s(path, slave_info_txt[i]);
			strcat_s(path, ".txt");
			err = fopen_s(&fs, path, "r");
			if (err != 0) {
				RtPrintf("Cannot open file: %s\n", path);
				return -1;
			}
			if (fs != nullptr) {
				if (i == 0) {
					fscanf(fs, "%i,%i,%i,%i,%i,%i\n", SERVO_DRIVER_ECAT_SLAVE_ID, SERVO_DRIVER_ECAT_SLAVE_ID + 1, SERVO_DRIVER_ECAT_SLAVE_ID + 2, SERVO_DRIVER_ECAT_SLAVE_ID + 3, SERVO_DRIVER_ECAT_SLAVE_ID + 4, SERVO_DRIVER_ECAT_SLAVE_ID + 5);
				}
				else {
					fscanf(fs, "%i,\n%i,\n", &ATI_FT_SENSOR_ECAT_SLAVE_ID, &ATI_FT_SENSOR_ECAT_IO_ID);
					RtPrintf("%s slave ID: %i\n%s I/O ID: %i\n", slave_info_txt[i], ATI_FT_SENSOR_ECAT_SLAVE_ID, slave_info_txt[i], ATI_FT_SENSOR_ECAT_IO_ID);
				}
				fclose(fs);
			}
			else {
				RtPrintf("Failed to open the file: %s.\n", path);
			}
		}
		//
		nRet = 1;
	}
	else {
		RtPrintf("Failed to open the file: %s.\n", path);
	}
	return nRet;
}

void RT_Robot_Controller::GetRtssAbsolutePath(_TCHAR* rtss_full_path) {
	//  rtss_full_path: 當前執行檔 *.rtss 的絕對路徑
	wcstombs_s(nullptr, app_dir, MAX_PATH, rtss_full_path, _TRUNCATE);
	char* pch = strrchr(app_dir, '\\');
	*pch = '\0'; // leave the file directory alone
	strcat(app_dir, "\\");
	//RtPrintf("Robot Controller .rtss absolute directory: %s\n", app_dir);
} 

int  RT_Robot_Controller::Initialize(void) {
	int ret = 0;
	SubsystemStatus Subsystem = { ecatOffline, ecatOffline, ETHERCAT_SLAVE_COUNT, ETHERCAT_IO_COUNT, ETHERCAT_SERVO_COUNT, {ecatOffline}, {ecatOffline}, {axisOffline} };
	KsCommandStatus Command = { 0 };
	KsError Code = errNoError;
#pragma region Initialization
	RtPrintf("Initializing the KSM subsystem ........\n");

	Code = Create(0, 0);
	if (Code != errNoError) {
		RtPrintf("Failed to create: 0x%x\n", Code);
		ret = -2;
		goto End;
	}
// Configure the subsystem settings
// Before enable the features, please check if the corresponding licenses are all enabled
// in the runtime environment.
#pragma region SUBSYSTEN_SETTING
	//Code = ConfigureDc(TRUE, TRUE, TRUE, 0);
	//if (Code != errNoError) {
	//	RtPrintf("Failed to ConfigureDc: 0x%x\n", Code);
	//	Destroy();
	//	ret = -2;
	//	goto End;
	//}
	Code = EnableAutoConfig(TRUE);
	if (Code != errNoError) {
		RtPrintf("Failed to EnableAutoConfig: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}
	Code = EnableAutoRepair(TRUE);
	if (Code != errNoError) {
		RtPrintf("Failed to EnableAutoRepair: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}
	Code = EnableAutoRestart(TRUE);
	if (Code != errNoError) {
		RtPrintf("Failed to EnableAutoRestart: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}
	//Code = EnableHotConnect(TRUE);

	// Disable logs in the RTX64 server console.
	Code = EnableServerLog(FALSE);
	if (Code != errNoError) {
		RtPrintf("Failed to set server log: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}
	RtPrintf("Subsystem configured\n");
#pragma endregion SUBSYSTEN_SETTING

#pragma region AXIS_SETTING
	// Configure the axis variables
// Before enabling variables in the program, you should:
// 1. Read the user manual to check if the variables are available in the device.
// 2. Check if the corresponding variables are put in the PDO list in ESI import tool.
// For further details, see Help System >> KINGSTAR ESI Import Tool >> Tabs >> PDO.
	// Configure the different modes used with the axes.
	Code = SetAxisAccessMode(accessPosVel);
	if (Code != errNoError) {
		RtPrintf("Failed to set access mode: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}

	// EtherCAT cycle time in seconds.
	Code = SetCycleTime(ETHERCAT_CYCLE_ms);
	if (Code != errNoError) {
		RtPrintf("Failed to set cycle time: 0x%x\n", Code);
		Destroy();
		ret = -2;
		goto End;
	}
	RtPrintf("Axes configured\n");

#pragma endregion AXIS_SETTING

	// Start the EtherCAT network
	// The timeout is set to 30s and the start command will be aborted if it is not done by then.
	// Check if the subsystem is started.
	// A new KINGSTAR subsystem is not started by default.
	Command = WaitForCommand(30, TRUE, Start());
	if (!Command.Done) {
		RtPrintf("Failed to start EtherCAT: 0x%x\n", Command.ErrorId);
		Destroy();
		ret = -2;
		goto End;
	}
	/*Code = GetStatus(&Subsystem, NULL);
	if (Subsystem.State == ecatOP)
	{
		RtPrintf("Subsystem already started: %x\n", Code);
	}
	else if (Code == errNoError && Subsystem.State == ecatOffline) {
		Command = WaitForCommand(30, TRUE, Start());
		if (!Command.Done) {
			RtPrintf("Failed to start EtherCAT: 0x%x\n", Command.ErrorId);
			Destroy();
			ret = -2;
			goto End;
		}
		RtPrintf("Subsystem Started\n");
	}*/
	Code = GetStatus(&Subsystem, NULL);
	if (Code != errNoError) {
		RtPrintf("Failed to get status: 0x%x\n", Code);
		ret = -1;
		goto End;
	}
	RtPrintf("Subsystem status\n  %d Slaves\n  %d I/Os\n  %d Axes\n\n", Subsystem.SlaveCount, Subsystem.IOCount, Subsystem.AxesCount);
	unsigned char CH0_EN[2] = { 1, 0 };
	unsigned char CH1_EN[2] = { 1, 0 };
	unsigned char CH2_EN[2] = { 1, 0 };
	unsigned char CH3_EN[2] = { 1, 0 };


	unsigned char sampling_rate[2] = { 1, 0 }; // 100KHz
	unsigned char range_mode[2] = { 1, 0 };   // -5~5V

	// Device initialize: PSD #1

	WriteIOSdoObject(7, 0x2002, 1, false, CH0_EN, 2);
	WriteIOSdoObject(7, 0x2002, 2, false, CH1_EN, 2);
	WriteIOSdoObject(7, 0x2002, 3, false, CH2_EN, 2);
	WriteIOSdoObject(7, 0x2002, 4, false, CH3_EN, 2);

	WriteIOSdoObject(7, 0x2000, 0, false, sampling_rate, 2);
	WriteIOSdoObject(7, 0x2001, 0, false, range_mode, 2);

	// Device initialize: PSD #2
	WriteIOSdoObject(8, 0x2002, 1, false, CH0_EN, 2);
	WriteIOSdoObject(8, 0x2002, 2, false, CH1_EN, 2);
	WriteIOSdoObject(8, 0x2002, 3, false, CH2_EN, 2);
	WriteIOSdoObject(8, 0x2002, 4, false, CH3_EN, 2);

	WriteIOSdoObject(8, 0x2000, 0, false, sampling_rate, 2);
	WriteIOSdoObject(8, 0x2001, 0, false, range_mode, 2);

#pragma endregion Initialization

#pragma region EtherCAT_Slave_figuration
	int target_slave_count = ETHERCAT_SLAVE_COUNT;
	if (Subsystem.SlaveCount != target_slave_count)
		RtPrintf("Configure %i slaves, detect %i slaves\n", target_slave_count, Subsystem.SlaveCount);

	int target_io_count = ETHERCAT_IO_COUNT;
	if (Subsystem.IOCount != target_io_count)
		RtPrintf("Configure %i I/O devices, detect %i I/O devices\n", target_io_count, Subsystem.IOCount);
#pragma endregion EtherCAT_Slave_figuration
	End:
	RtPrintf("Finishing intialization.\n");
	return ret;
}

int RT_Robot_Controller::stopEtherCAT(void) {
	// Stop the EtherCAT network before destroying KINGSTAR Subsystem or before re-configuration.
	
	KsCommandStatus Command = { 0 };
	Command = WaitForCommand(5, FALSE, Stop());
	if (Command.Error)
	{
		RtPrintf("Stop Failed: 0x%x\n", Command.ErrorId);
		return -1;
	}
	else {
		RtPrintf("Stop EtherCAT network successful\n");
	}

	return 0;
}

int RT_Robot_Controller::destroyLink(void) {
	// Terminates the KINGSTAR Subsystem if there is no other application connected to it.
	KsError nRet = errNoError;
	nRet = Destroy();
	if (nRet != errNoError)
	{
		RtPrintf("Destroy Failed: 0x%x\n", nRet);
	}
	else {
		RtPrintf("Destroy KINGSTAR Subsystem successe\n");
	}
	return nRet;
}

void RT_Robot_Controller::getIOState(void) {
	SlaveStatus slaveStatus = { 0 };
	KsError nRet;
	for (int i = 0; i < 9; i++) {

		nRet = GetIOByIndex(i, &slaveStatus);
		if (nRet == errNoError)
		{
			// Check the slave information in the SlaveStatus structure.
			RtPrintf("IO %d %s: Vendor 0x%x, Product 0x%x, Revision 0x%x, Serial 0x%x\n", i,
				slaveStatus.Name, slaveStatus.VendorId, slaveStatus.ProductCode,
				slaveStatus.RevisionNumber, slaveStatus.SerialNumber);
			RtPrintf("Address: Auto %d, Fixed %d, Alias %d\n",
				slaveStatus.SlaveId, slaveStatus.PhysAddress, slaveStatus.AliasAddress);
			RtPrintf("PDO: Input len %d, Output len %d, Index offset 0x%x\n",
				slaveStatus.InputLength, slaveStatus.OutputLength, slaveStatus.VariableIndexOffset);
			RtPrintf("State %d, Cycle time %d\n", slaveStatus.State, slaveStatus.CycleTime);
		}
	}
	int resolution = 0;
	DWORD InputVar = 0, OutputVar = 0;
	for (int i = 0; i < 6; i++) {
		nRet = GetAxisByIndex(i, &slaveStatus, &resolution, &InputVar, &OutputVar);
		if (nRet == errNoError)
		{
			// Check the slave information in the SlaveStatus structure.
			RtPrintf("Axis 0 %s: Vendor 0x%x, Product 0x%x, Revision 0x%x, Serial 0x%x\n",
				slaveStatus.Name, slaveStatus.VendorId, slaveStatus.ProductCode,
				slaveStatus.RevisionNumber, slaveStatus.SerialNumber);
			RtPrintf("Address: Auto %d, Fixed %d, Alias %d\n",
				slaveStatus.SlaveId, slaveStatus.PhysAddress, slaveStatus.AliasAddress);
			RtPrintf("PDO: Input len %d, Output len %d, Index offset 0x%x\n",
				slaveStatus.InputLength, slaveStatus.OutputLength, slaveStatus.VariableIndexOffset);
			RtPrintf("State %d, Cycle time %d\n", slaveStatus.State, slaveStatus.CycleTime);

			// Use defined bit mask to check if desired variables are in the PDO data.
			if ((InputVar & VAR_MOP_DISPLAY) == VAR_MOP_DISPLAY) {
				BYTE mopDisplay = 0;
				nRet = ReadAxisMopDisplay(0, &mopDisplay);
				if (mopDisplay != 8 && (OutputVar & VAR_MOP) == VAR_MOP) {
					nRet = WriteAxisMop(0, 8);
				}
			}
		}
	}
}

#pragma endregion ConnectionSetting

void RT_Robot_Controller::ImportRobotParameter(char* app_dir) {
	int servo_indice[6] = { 0, 1, 2, 3, 4, 5 };
	RtPrintf("Import robot parameters.\n");
	//rt605.kinePrt.ImportAllParameter(app_dir);
	bool ret = rt605.kinePrt.ImportAllParameterJSON(app_dir);
	if (ret == false) {
		RtPrintf("Import Robot Parameters failed\n");
	}
	// 單位轉換，運動極限與Home點有關之參數，從 rad 轉換成 pulse：
	double NOT_pulse[6], POT_pulse[6];
	// 第三軸~第第六軸需要反向：
	rt605.JointToCount(rt605.kinePrt.joint_limit.at(0).data(), NOT_pulse);
	rt605.JointToCount(rt605.kinePrt.joint_limit.at(1).data(), POT_pulse);
	swap<double>(NOT_pulse + 2, POT_pulse + 2, 4);
	// 設定 KSM 系統中的角度軟體極限：
	KsCommandStatus parameterCommand;
	for (int i = 0; i < 6; i++) {
		parameterCommand = WaitForCommand(5, TRUE,
			SetAxisParameter(i, McAxisParameter::mcSoftLimitPositive, 6, McExecutionMode::mcImmediately));
		parameterCommand = WaitForCommand(5, TRUE,
			SetAxisParameter(i, McAxisParameter::mcSoftwareLimitNegative, 6, McExecutionMode::mcImmediately));
		if (parameterCommand.Error) {
			RtPrintf("Set Axis Parameter software limit failed: 0x%x\n", parameterCommand.ErrorId);
		}
	}
	//ConfigServoParam(Param::paramMinLimit, 6, servo_indice, NOT_pulse);
	//ConfigServoParam(Param::paramMaxLimit, 6, servo_indice, POT_pulse);
	// 
	// 載入 Robot Model "rt605" 至 Robot Controller 中的 MotionFeedback 模組：(用於內部運動學與單位的轉換)
	pShared_feedback->set_RobotModel(&rt605);
	feedback[0].set_RobotModel(&rt605);
	feedback[1].set_RobotModel(&rt605);


	// 設定初始速度與加速度：
	OV_Speed = 10.0;
	
	KsError nRet;
	for (int i = 0; i < 6; ++i) {
		//SetServoMotionProfileType(i, profileDelayInSecond);
		servo_profile[i].MinimumFollowingError = 3;
		servo_profile[i].MaximumFollowingError = 36000;
		servo_profile[i].MaximumVelocity = abs(deg2rad(OV_Speed*6) * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU);

		//if (i >= 2) {
		//	//servo_profile[i].MinimumVelocity = deg2rad(0.0) * -rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		//	servo_profile[i].MaximumVelocity = deg2rad(30.0) * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		//}
		//else {
		//	//servo_profile[i].MinimumVelocity = deg2rad(0.0) * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		//	servo_profile[i].MaximumVelocity = deg2rad(30.0) * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		//}
		servo_profile[i].Acceleration = 0.1; // 1 second
		servo_profile[i].Deceleration = 0.1; // 1 second
		servo_profile[i].Jerk = 0.01;
		//servo_profile[i].DecelerationJerk = 0.08;
		servo_profile[i].Jolt = 0.0;
		nRet = SetAxisMotionProfile(i, McProfileType::profileDelayInSecond, servo_profile[i]);
		if (nRet!=0) {
			RtPrintf("SetAxisMotionProfile failed: 0x%x\n", nRet);
			return;
		}
		//ConfigServoMotion(i, servo_profile[i]);
	}

	this->mechanism_analyzer.set_app_dir(this->app_dir);
	this->sinusoidal_test.set_app_dir(this->app_dir);
	
	RtPrintf("SetAxisMotionProfile Success\n");
}
//
int RT_Robot_Controller::ServoON(void) {
    /* Return -1 if faild 0 if success*/
	KsCommandStatus status;
    for (int Index = 0; Index < 6; Index++) {
        /*status = WaitForCommand(5, TRUE, ResetAxis(Index));
        status = WaitForCommand(5, TRUE, PowerAxis(Index, TRUE, TRUE, TRUE));
        if (status.Error) {
            return -1;
        }*/
		status = PowerAxis(Index, TRUE, TRUE, TRUE);
		if (status.Error) {
			return -1;
		}
    }
    return 0;
}
//
int RT_Robot_Controller::ServoOFF(void) {
    /* Return -1 if faild 0 if success*/
	KsCommandStatus status;
    for (int Index = 0; Index < 6; Index++) {
        status = WaitForCommand(5, TRUE, ResetAxis(Index));
        status = PowerAxis(Index, FALSE, FALSE, FALSE);
        if (status.Error) {
            return -1;
        }
    }
    return 0;
}
//
int  RT_Robot_Controller::LoadTaskProgram(char* intp_file) {
	// 任務專案的名稱，例如："exp1_20220906"
	int ret = 0;
	FILE* fs = nullptr;
	errno_t err;
	char open_file_tmp[MAX_PATH] = "";
	// 設定要讀取的 INTP 檔之絕對路徑：
	memset(open_file_tmp, NULL, MAX_PATH);
	strncpy(open_file_tmp, this->app_dir, MAX_PATH);
	strcat(open_file_tmp, "Parameters\\task\\");
	strcat_s(open_file_tmp, 128, intp_file);
	strcat(open_file_tmp, "\\");
	strcat_s(open_file_tmp, 128, intp_file);
	strcat(open_file_tmp, ".txt");
	// 開始讀取 INTP 檔：
	ret = ReadHrssIntpFile(open_file_tmp);
	if (ret != 0) {
		RtPrintf("LoadTaskProgram failed\n");
		return -1;
	}
	// 讀取 Task 之 critical interval：
	
	//memset(open_file_tmp, NULL, MAX_PATH);
	//strncpy(open_file_tmp, this->app_dir, MAX_PATH); 
	//strcat(open_file_tmp, "Parameters\\task\\");
	//strcat_s(open_file_tmp, 128, intp_file);
	//strcat(open_file_tmp, "\\");
	//strcat(open_file_tmp, "fc_critical_interval.txt");

	//errno_t err = fopen_s(&fs, open_file_tmp, "r");
	//
	//if (fs != nullptr) {
	//	unsigned long long fc_start, fc_stop, process_index[2];
	//	fscanf(fs, "%i,%i,%i", &fc_start, process_index, process_index + 1);
	//	this->fc_start_point = fc_start;
	//	this->fc_process_phrase[0] = process_index[0];
	//	this->fc_process_phrase[1] = process_index[1];
	//	//fcImp1.setControlProcessSectoin(fc_start, fc_stop, process_index[0], process_index[1]);
	//	RtPrintf("Force control, the fc_start index: %i\n", fc_start_point);
	//	RtPrintf("Force control, critical section: %i ~ %i.\n", fc_process_phrase[0], fc_process_phrase[1]);
	//	fclose(fs);
	//}
	//else {
	//	RtPrintf("Failed to open file: %s.\n", open_file_tmp);
	//}
	// 讀取 Task 之工具坐標系：
	// (3)	從力量感測器坐標系轉換置TCP座標系
	//memset(open_file_tmp, NULL, MAX_PATH);
	//strncpy(open_file_tmp, this->app_dir, MAX_PATH);
	//strcat(open_file_tmp, "Parameters\\task\\");
	//strcat_s(open_file_tmp, 128, intp_file);
	//strcat(open_file_tmp, "\\");
	//strcat(open_file_tmp, "sensor_tcp.txt");
	//fopen_s(&fs, open_file_tmp, "r");
	//if (fs != nullptr) {
	//	double transform[6];
	//	fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", &transform[0], &transform[1], &transform[2], &transform[3], &transform[4], &transform[5]);
	//	fclose(fs);
	//	// 單位轉換：
	//	transform[0] = transform[0] / 1000.0; // Dx (unit: m)
	//	transform[1] = transform[1] / 1000.0; // Dy (unit: m)
	//	transform[2] = transform[2] / 1000.0; // Dz (unit: m)
	//	transform[3] = deg2rad(transform[3]); // Rx (unit: rad)
	//	transform[4] = deg2rad(transform[4]); // Ry (unit: rad)
	//	transform[5] = deg2rad(transform[5]); // Rz (unit: rad)
	//	m_fts.setTransformSensorToTCP(transform);
	//}
	//else {
	//	RtPrintf("Failed to open file: %s.\n", open_file_tmp);
	//}

	// Read task information "workspace_frame.txt":
	// (2)	從機械手臂參考坐標系轉換至新的工作區坐標系
	fs = nullptr;
	memset(open_file_tmp, NULL, MAX_PATH);
	strncpy(open_file_tmp, this->app_dir, MAX_PATH);
	strcat(open_file_tmp, "Parameters\\task\\");
	strcat_s(open_file_tmp, 128, intp_file);
	strcat(open_file_tmp, "\\");
	strcat_s(open_file_tmp, 128, "workspace_frame.txt");
	err = fopen_s(&fs, open_file_tmp, "r");
	if (err != 0) {
		RtPrintf("open file %s failed...\n", open_file_tmp);
		return -1;
	}
	if (fs != nullptr) {
		double task_frame_tmp[6];
		fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", task_frame_tmp, task_frame_tmp + 1, task_frame_tmp + 2, task_frame_tmp + 3, task_frame_tmp + 4, task_frame_tmp + 5);
		fclose(fs);
		// 單位轉換：
		task_frame_tmp[0] = task_frame_tmp[0] / 1000.0; // Dx (unit: m)
		task_frame_tmp[1] = task_frame_tmp[1] / 1000.0; // Dy (unit: m)
		task_frame_tmp[2] = task_frame_tmp[2] / 1000.0; // Dz (unit: m)
		task_frame_tmp[3] = deg2rad(task_frame_tmp[3]); // Rx (unit: rad)
		task_frame_tmp[4] = deg2rad(task_frame_tmp[4]); // Ry (unit: rad)
		task_frame_tmp[5] = deg2rad(task_frame_tmp[5]); // Rz (unit: rad)
		rt605.set_WorkspaceFrame(task_frame_tmp);
	}

	//(1)	從RT605 - 710末端法蘭面轉換至TCP的坐標系
	fs = nullptr;
	memset(open_file_tmp, NULL, MAX_PATH);
	strncpy(open_file_tmp, this->app_dir, MAX_PATH);
	strcat(open_file_tmp, "Parameters\\task\\");
	strcat_s(open_file_tmp, 128, intp_file);
	strcat(open_file_tmp, "\\");
	strcat_s(open_file_tmp, 128, "tool_frame.txt");
	err = fopen_s(&fs, open_file_tmp, "r");
	if (err != 0) {
		RtPrintf("open file %s failed...\n", open_file_tmp);
		return -1;
	}
	if (fs != nullptr) {
		double task_frame_tmp[6];
		fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf", task_frame_tmp, task_frame_tmp + 1, task_frame_tmp + 2, task_frame_tmp + 3, task_frame_tmp + 4, task_frame_tmp + 5);
		fclose(fs);
		// 單位轉換：
		task_frame_tmp[0] = task_frame_tmp[0] / 1000.0; // Dx (unit: m)
		task_frame_tmp[1] = task_frame_tmp[1] / 1000.0; // Dy (unit: m)
		task_frame_tmp[2] = task_frame_tmp[2] / 1000.0; // Dz (unit: m)
		task_frame_tmp[3] = deg2rad(task_frame_tmp[3]); // Rx (unit: rad)
		task_frame_tmp[4] = deg2rad(task_frame_tmp[4]); // Ry (unit: rad)
		task_frame_tmp[5] = deg2rad(task_frame_tmp[5]); // Rz (unit: rad)
		rt605.set_TcpFrame(task_frame_tmp);
	}
	else {
		RtPrintf("Failed to open file: %s.\n", open_file_tmp);
	}

	return ret;
}
//
int RT_Robot_Controller::ReadHrssIntpFile(const char* _file) {
	// HRSS intp 檔的絕對路徑
	// ----------------------------- 開始計算軌跡的插補點數量：-----------------------------
	FILE* fs = nullptr;
	errno_t err;
	err = fopen_s(&fs, _file, "r");
	if (err != 0 || fs == nullptr) {
		RtPrintf("Open file %s Error\n", _file);
		return -1;
	}
	else {
		RtPrintf(" - Open the file stream and load the INTP: ' %s '->Xr_buffer\n", _file);
		// 計算總資料長度：
		Xr_length = 0;
		double tmp[34];
		/* 一個 column 一個 column 掃描，計算一筆 sample 有幾個欄位的資料，
			如果結果為 col_num == 12 代表現在讀的 INTP_LOG 是從 HRSS_offline 產生的；結果為 col_num == 34 則代表現在讀的 INTP_LOG 是從 HRSS_online 產生的。
		*/
		unsigned int col_num = 1;
		for (int c = getc(fs); c != '\n'; c = getc(fs)) {
			if (c == ',')
				++col_num;
		}
		++Xr_length;
		for (int c = getc(fs); !feof(fs); c = getc(fs)) {
			if (c == '\n')
				++Xr_length;
		}
		fclose(fs);
		RtPrintf(" - The amout of interpolation point: %i.\n", Xr_length);
		RtPrintf("%d\n", col_num);

		// ----------------------------- 動態配置記憶體：-----------------------------
		if (Xr_buf == nullptr && Xr_buf_heap_capacity == 0) { // for the first time
			Xr_buf = (double**)calloc(Xr_length, sizeof(double*));
			for (unsigned long long i = 0; i < Xr_length; ++i)
				Xr_buf[i] = (double*)calloc(12, sizeof(double));
			// 紀錄目前已經配置在 Heap 區的記憶體長度：
			Xr_buf_heap_capacity = Xr_length;
			RtPrintf(" - The INTP buffer has been allocated. Buffer size: %u, Heap capacity: %u\n", Xr_length, Xr_buf_heap_capacity);
		}
		else if (Xr_buf != nullptr && Xr_length > Xr_buf_heap_capacity) {
			// Release the heap:
			for (unsigned long long i = 0; i < Xr_buf_heap_capacity; ++i) {
				free(Xr_buf[i]);
				Xr_buf[i] = nullptr;
			}
			free(Xr_buf); Xr_buf = nullptr;
			// Re-allocate the heap:
			Xr_buf = (double**)calloc(Xr_length, sizeof(double*));
			for (unsigned long long i = 0; i < Xr_length; ++i)
				Xr_buf[i] = (double*)calloc(12, sizeof(double));
			// 紀錄目前已經配置在 Heap 區的記憶體長度：
			Xr_buf_heap_capacity = Xr_length;
			RtPrintf(" - The INTP buffer has been re-allocated. Buffer size: %u, Heap capacity: %u\n", Xr_length, Xr_buf_heap_capacity);
		}
		else if (Xr_buf != nullptr && Xr_length <= Xr_buf_heap_capacity) {

			/*
			* Do nothing.
			*/

			RtPrintf(" - The INTP buffer has been re-allocated.\n Buffer size: %u, Heap capacity: %u\n", Xr_length, Xr_buf_heap_capacity);
		}
		// 開始載入插補點：
		err = fopen_s(&fs, _file, "r");
		if (err != 0) {
			RtPrintf("Open file %s Error\n", _file);
			return -1;
		}
		for (unsigned long long i = 0; i < Xr_length; ++i) {
			if (col_num == 34) { // 所選擇的 HRSS INTP_LOG 檔是從實際機台 (HRSS_online) 產生的：
				fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
							%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
							%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,\
							%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
					tmp,
					Xr_buf[i],
					Xr_buf[i] + 1,
					Xr_buf[i] + 2,
					Xr_buf[i] + 3,
					Xr_buf[i] + 4,
					Xr_buf[i] + 5,
					Xr_buf[i] + 6,
					Xr_buf[i] + 7,
					Xr_buf[i] + 8,
					Xr_buf[i] + 9,
					Xr_buf[i] + 10,
					Xr_buf[i] + 11,
					tmp + 13,
					tmp + 14,
					tmp + 15,
					tmp + 16,
					tmp + 17,
					tmp + 18,
					tmp + 19,
					tmp + 20,
					tmp + 21,
					tmp + 22,
					tmp + 23,
					tmp + 24,
					tmp + 25,
					tmp + 26,
					tmp + 27,
					tmp + 28,
					tmp + 29,
					tmp + 30,
					tmp + 31,
					tmp + 32,
					tmp + 33);
			}
			else { // 所選擇的 HRSS INTP_LOG 檔是從軟體 HRSS_offline 產生的：
				fscanf(fs, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", \
					Xr_buf[i],
					Xr_buf[i] + 1,
					Xr_buf[i] + 2,
					Xr_buf[i] + 3,
					Xr_buf[i] + 4,
					Xr_buf[i] + 5,
					Xr_buf[i] + 6,
					Xr_buf[i] + 7,
					Xr_buf[i] + 8,
					Xr_buf[i] + 9,
					Xr_buf[i] + 10,
					Xr_buf[i] + 11,
					tmp + 12,
					tmp + 13,
					tmp + 14);
			}
			// 將 HRSS INTP LOG 中定義的 (A, B, C) Command 與 Joint Command 之單位從 (m-degree) 轉換成 (radian)：
			for (unsigned short j = 0; j < 12; ++j) {
				if (j > 2) // 單位轉換成 rad.
					Xr_buf[i][j] = deg2rad(Xr_buf[i][j] / 1000.0);
				else // 單位轉換成 m
					Xr_buf[i][j] = Xr_buf[i][j] / 1000000.0;
			}
		}
		fclose(fs);
		// 印出隨意 2 組 sample 的值觀察一下：
		RtPrintf("[%d] = ", 0);
		for (int i = 0; i < 12; ++i) {
			if (i < 3)
				RtPrintf("%d  ", (int)(Xr_buf[0][i] * 1000000.0));
			else
				RtPrintf("%d  ", (int)(rad2deg(Xr_buf[0][i])));
		}
		double q[6];
		double x1[6] = { Xr_buf[0][0], Xr_buf[0][1] , Xr_buf[0][2] , Xr_buf[0][3] , Xr_buf[0][4], Xr_buf[0][5] };
		//rt605.RT605_IK(x1, Robot_Postures::MarkUp_ElbowUp_WristFronted, q, UNIT_DEGREE);
		rt605.RT605_IK_Config1(x1, q);
		RtPrintf("\n%i, %i, %i, %i, %i, %i\n", (int)rad2deg(q[0]), (int)rad2deg(q[1]), (int)rad2deg(q[2]), (int)rad2deg(q[3]), (int)rad2deg(q[4]), (int)rad2deg(q[5]));
		rt605.RT605_IK(x1, Robot_Postures::MarkUp_ElbowUp_WristFlipped, q, UNIT_DEGREE);
		RtPrintf("\n%i, %i, %i, %i, %i, %i\n", (int)q[0], (int)q[1], (int)q[2], (int)q[3], (int)q[4], (int)q[5]);
		RtPrintf("\n\n");
		unsigned int k = (unsigned int)(((double)Xr_length) * 0.65);
		RtPrintf("[%d] = ", k);
		for (int i = 0; i < 12; ++i) {
			if (i < 3)
				RtPrintf("%d  ", (int)(Xr_buf[k][i] * 1000000.0));
			else
				RtPrintf("%d  ", (int)(rad2deg(Xr_buf[k][i])));
		}
		double x2[6] = { Xr_buf[k][0], Xr_buf[k][1], Xr_buf[k][2] , Xr_buf[k][3] , Xr_buf[k][4], Xr_buf[k][5] };
		rt605.RT605_IK(x2, Robot_Postures::MarkUp_ElbowUp_WristFronted, q, UNIT_DEGREE);
		RtPrintf("\n%i, %i, %i, %i, %i, %i\n", (int)q[0], (int)q[1], (int)q[2], (int)q[3], (int)q[4], (int)q[5]);
		rt605.RT605_IK(x2, Robot_Postures::MarkUp_ElbowUp_WristFlipped, q, UNIT_DEGREE);
		RtPrintf("\n%i, %i, %i, %i, %i, %i\n", (int)q[0], (int)q[1], (int)q[2], (int)q[3], (int)q[4], (int)q[5]);
		RtPrintf("\n\n");
	}
	return 0;
	/* Example: */
/* test_intp.cpp
// *****************************************************************************

#include<rc_utility_intp_loader.h"

int main(int argc, char* argv[]){
   std::pair<unsigned long long, double**> Xr_buf;

   // 讀 HRSS INTP LOG 檔 "intp.txt"：
   ReadHrssIntpFile("./intp.txt", Xr_buf);

   // 讀 HRSS INTP LOG 檔 "intp2.txt"，此時會重新配置記憶體 (日後可以修改成 memory pool 的方式避免因為多次的讀檔造成記憶體破碎)：
   ReadHrssIntpFile("./intp2.txt", Xr_buf);

   return 0;
}

// *****************************************************************************
*/
}
void RT_Robot_Controller::StartINTPtask(void) {
	//std::array<double, 6> q;
	double q[6];
	memcpy(q, &Xr_buf[0][6], 6 * sizeof(double));
	rt605.JointToCount(q);
	for (int i = 0; i < 6; ++i) {
		SetAxisControlMode(i, McControlMode::modeMasterIntPos);
		//SetServoControlMode(i, ControlMode::modeMasterIntPos);
		//DefineServoAbsolute(i, q[i]); Point to point interpolated move with an absolute target. The axis will move when MoveServo is called. Read the motion status to see when the target is reached.
	}
	//Move servo to start position of interpolation position
	int ret = MoveJointAbs(q);
	if (ret != 0) {
		RtPrintf("Move joints to start position of INTP failed...\n");
	}
	//int axes[6] = { 0, 1, 2, 3, 4, 5 };
	//MoveServo(6, axes);
	RtSleep(10);
	McDirection mcDirection = McDirection::mcPositiveDirection;
	while (system_state->servoMotion_state[0] != McMotionState::mcHalted || system_state->servoMotion_state[1] != McMotionState::mcHalted || system_state->servoMotion_state[2] != McMotionState::mcHalted ||
		system_state->servoMotion_state[3] != McMotionState::mcHalted || system_state->servoMotion_state[4] != McMotionState::mcHalted || system_state->servoMotion_state[5] != McMotionState::mcHalted)
	{
		for (int i = 0; i < 6; ++i)
			GetAxisMotionState(i, system_state->servoMotion_state.data() + i, &mcDirection);
		RtSleep(10);
	}
	RtPrintf(" - Cycle start.\n");
	// 切換至 CSP 模式：
	for (int i = 0; i < 6; ++i) {
		SetAxisControlMode(i, McControlMode::modeDirectPos);
		//SetServoControlMode(i, ControlMode::modeDirectPos);
	}
	// 初始化控制器各狀態：
	xr_index = 0;
	StartPositionMotionRoutine();
}

void RT_Robot_Controller::TerminateCspTask(void) {
	for (int i = 0; i < 6; ++i) {
		SetAxisControlMode(i, McControlMode::modeMasterIntPos);
		//SetServoControlMode(i, ControlMode::modeMasterIntPos);
	}
	RtCancelTimer(hTimer_motion, NULL);
	//RtCancelTimer(hTimer_fc_motion, NULL);
	//fcImp1.ResetState();
	RtPrintf("TerminateCspTask");
}

//
int RT_Robot_Controller::SetRobotTargetPosition(double* _pos) {
	int ret = 0;
	double qc[6];
	// 關節姿態解的選擇：
	if (feedback[0].q[4] >= 0.0)
		rt605.RT605_IK_Config1(_pos, qc);
	else
		rt605.RT605_IK_Config2(_pos, qc);
	//
	ret = SetRobotJointTargetPosition(qc);
	return ret;
}

int RT_Robot_Controller::SetRobotJointTargetPosition(double* _q) {
	// _q: csp 模式下要送至6軸驅動器的角度命令 (角度單位為 rad)，例如：double qr[6] = {0.0, 0.0, 0.0, 0.0, -pi/2, 0.0};
	return 0;
}

int RT_Robot_Controller::MoveRobotAbsPTP(unsigned short _frame, double* _pos) {
	// reserved
	RtPrintf("MoveRobotAbsPTP Not Implement Yet\n");
	return 0;
}

int RT_Robot_Controller::MoveRobotRelPTP(unsigned short _frame, double* _pos){
	// reserved
	RtPrintf("MoveRobotRelPTP Not Implement Yet\n");
	return 0;
}
 
//int MoveRobotAbsLINE(unsigned short _frame, double* _pos);
//int MoveRobotRelLINE(unsigned short _frame, double* _pos);


int RT_Robot_Controller::MoveJointRel(int _index, double _q) {
	// _index: 要移動的關節的index；_q: 要移動的相對角度 (角度單位為 rad)
	int ret = 0;
	KsCommandStatus status;
	double joint_speed=0.0;
	if (system_state->isServoON())
	{
		for (int i = 0; i < 6; ++i)
			SetAxisControlMode(i, McControlMode::modeMasterIntPos);
		double&& motor_tmp = _q * ((double)(rt605.kinePrt.reduction_ratio.at(_index)) * ((double)rt605.kinePrt.PPU));
		
		if (_index == 4) { // move joint 5
			//m[5] = (q[5] * -50) - q[4];
			double&& motor_tmp6 = -_q * ((double)rt605.kinePrt.PPU);
			joint_speed = abs(deg2rad(OV_Speed) * rt605.kinePrt.reduction_ratio[4] * rt605.kinePrt.PPU);
			//Start an relative move.
			status = MoveAxisRelative(4, motor_tmp,
				joint_speed, servo_profile[4].Acceleration,
				servo_profile[4].Deceleration, servo_profile[4].Jerk, mcAborting);
			joint_speed = abs(deg2rad(OV_Speed) * rt605.kinePrt.reduction_ratio[5] * rt605.kinePrt.PPU);
			status = MoveAxisRelative(5, motor_tmp,
				joint_speed, servo_profile[5].Acceleration,
				servo_profile[5].Deceleration, servo_profile[5].Jerk, mcAborting);
			//status = WaitForCommand(30, TRUE, MoveAxisRelative(4, motor_tmp,
			//	servo_profile[4].Acceleration / 6, servo_profile[4].Acceleration, 
			//	servo_profile[4].Deceleration, servo_profile[4].Jerk, mcAborting));
			//status = WaitForCommand(30, TRUE, MoveAxisRelative(5, motor_tmp,
			//	servo_profile[5].Acceleration / 6, servo_profile[5].Acceleration, 
			//	servo_profile[5].Deceleration, servo_profile[5].Jerk, mcAborting));
		}
		else { // joint 1~4 and joint 6
			joint_speed = abs(deg2rad(OV_Speed) * rt605.kinePrt.reduction_ratio[_index] * rt605.kinePrt.PPU);
			status = MoveAxisRelative(_index, motor_tmp,
				joint_speed, servo_profile[_index].Acceleration,
				servo_profile[_index].Deceleration, servo_profile[_index].Jerk, mcAborting);
			//status = WaitForCommand(30, TRUE, MoveAxisRelative(_index, motor_tmp,
			//	servo_profile[_index].Acceleration / 6, servo_profile[_index].Acceleration, 
			//	servo_profile[_index].Deceleration, servo_profile[_index].Jerk, mcAborting));
		}
	}
	else {
		RtPrintf("\nFailed: The robot hasn't servo on yet.\n");
	}
	return ret;
}

int RT_Robot_Controller::MoveJointRel(double* _q) {
	// _q: 6軸要移動的相對角度 (角度單位為 rad)
	int ret = 0;
	KsCommandStatus status;
	double joint_speed;
	for (int _index = 0; _index < 6; _index++) {
		joint_speed = abs(deg2rad(OV_Speed) * rt605.kinePrt.reduction_ratio[_index] * rt605.kinePrt.PPU);
		status = MoveAxisRelative(_index, _q[_index], joint_speed, servo_profile[_index].Acceleration,
			servo_profile[_index].Deceleration, servo_profile[_index].Jerk, mcAborting);
		//status = WaitForCommand(30, TRUE, MoveAxisRelative(_index, _q[_index],
		//	OV_Speed, servo_profile[_index].Acceleration, servo_profile[_index].Deceleration, servo_profile[_index].Jerk, mcAborting));
		if (status.Error) {
			RtPrintf("MoveAxisRelative failed Error ID: 0x%x\n", status.ErrorId);
			return -1;
		}
	}
	
	return ret;
}

int RT_Robot_Controller::MoveJointAbs(double* _q) {
	// _q: 6軸要移動的絕對角度 (角度單位為 rad)
	KsCommandStatus status;
	double joint_speed;
	for (int _index = 0; _index < 6; _index++) {
		joint_speed = abs(deg2rad(OV_Speed) * rt605.kinePrt.reduction_ratio[_index] * rt605.kinePrt.PPU);
		status = MoveAxisAbsolute(_index, _q[_index], joint_speed,
			servo_profile[_index].Acceleration, servo_profile[_index].Deceleration, servo_profile[_index].Jerk, McDirection::mcShortestWay, mcAborting);
		if (status.Error) {
			RtPrintf("MoveAxisAbsolute failed Error ID: 0x%x\n", status.ErrorId);
			return -1;
		}
		//status = WaitForCommand(30, TRUE, MoveAxisAbsolute(_index, _q[_index],
		//	servo_profile[_index].Acceleration / 6, servo_profile[_index].Acceleration,
		//	servo_profile[_index].Deceleration, servo_profile[_index].Jerk, McDirection::mcShortestWay ,mcAborting));
		//if (status.Error) {
		//	RtPrintf("MoveAxisAbsolute failed Error ID: 0x%x\n", status.ErrorId);
		//	return -1;
		//}
	}
	
	return 0;
}

int RT_Robot_Controller::MoveJointJog(int _index, double _vel) {
	// _index: 要移動的關節的index；_vel: JOG的轉速 (單位為 rad/sec.)
	McDirection direction = mcPositiveDirection;
	KsCommandStatus status;
	
	if (system_state->isServoON()) // 確認各軸都已經 Servo ON：
	{
		for (int i = 0; i < 6; ++i)
			status = SetAxisControlMode(i, McControlMode::modeMasterIntPos);
		if (status.Error) {
			RtPrintf("SetAxisControlMode failed Error ID: 0x%x\n", status.ErrorId);
			return -1;
		}
		double&& motor_vel_tmp = _vel * ((double)(rt605.kinePrt.reduction_ratio.at(_index)) * ((double)rt605.kinePrt.PPU));
		if (motor_vel_tmp < 0) {
			direction = mcNegativeDirection;
			motor_vel_tmp = -1*motor_vel_tmp;
		}
			
		char speed_str[32] = { 0 };
		sprintf(speed_str, "%f", motor_vel_tmp);
		RtPrintf("Jog Speed %s\n", speed_str);

		// move joint 
		if (_index == 4) { // move joint 5
				//m[5] = (q[5] * -50) - q[4];
			double&& motor_vel_tmp6 = -1 * _vel * ((double)rt605.kinePrt.PPU);
			McDirection direction_tmp6 = (direction == mcPositiveDirection) ?  mcNegativeDirection : mcPositiveDirection;
			//Start the velocity motion
			JogAxis(4, motor_vel_tmp, servo_profile[4].Acceleration, servo_profile[4].Deceleration, servo_profile[4].Jerk, direction);
			/*status = WaitForCommand(5, FALSE, JogAxis(4, motor_vel_tmp,
				servo_profile[4].Acceleration, servo_profile[4].Deceleration, servo_profile[4].Jerk, direction));
			if (status.Error) {
				RtPrintf("Jg Axis 4 failed Error ID: 0x%x\n",status.ErrorId);
				return -1;
			}*/
			JogAxis(5, motor_vel_tmp6, servo_profile[5].Acceleration, servo_profile[5].Deceleration, servo_profile[5].Jerk, direction_tmp6);
			/*status = WaitForCommand(5, FALSE, JogAxis(5, motor_vel_tmp,
				servo_profile[5].Acceleration, servo_profile[5].Deceleration, servo_profile[5].Jerk, direction));
			if (status.Error) {
				RtPrintf("Jg Axis 5 failed Error ID: 0x%x\n", status.ErrorId);
				return -1;
			}*/
		}
		else { // joint 1~4 and joint 6
			JogAxis(_index, motor_vel_tmp, servo_profile[_index].Acceleration, servo_profile[_index].Deceleration, servo_profile[_index].Jerk, direction);
			//status = WaitForCommand(5, FALSE, JogAxis(_index, motor_vel_tmp,
			//	servo_profile[_index].Acceleration, servo_profile[_index].Deceleration, servo_profile[_index].Jerk, direction));
			//if (status.Error) {
			//	RtPrintf("Jg Axis %d failed Error ID: 0x%x\n", _index, status.ErrorId);
			//	return -1;
			//}
		}
	}
	else {
		RtPrintf("\nFailed: The robot hasn't servo on yet.\n");
		return -1;
	}
	return 0;
}

void RT_Robot_Controller::SetOVSpeed(double _vel) {
	this->OV_Speed = _vel; // Unit: deg/second
}

int RT_Robot_Controller::HaltMove(int Index) {
	//KsCommandStatus status = WaitForCommand(5, FALSE, HaltAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk, mcAborting));
	KsCommandStatus status = HaltAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk, mcAborting);
	if (status.Error) {
		RtPrintf("Axis %d HaltMove Error: 0x%x\n", Index, status.ErrorId);
		return - 1;
	}
	return 0;
}

int RT_Robot_Controller::HaltAll(void) {
	KsCommandStatus status;
	for (int Index = 0; Index < 6; Index++) {
		status = HaltAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk, mcAborting);
		if (status.Error) {
			RtPrintf("Axis %d HaltMove Error: 0x%x\n", Index, status.ErrorId);
			return -1;
		}
	}
	return 0;
}

int RT_Robot_Controller::StopMove(void) {
	int ret = 0;
	KsCommandStatus status;
	for (int Index = 0; Index < 6; ++Index) {
		//StopAxis will halt and lock the axis preventing any other move to run
		status = StopAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk);
		if (status.Error == TRUE) {
			RtPrintf("StopAxis Error!! Error ID:0x%x\n", status.ErrorId);
		}
		//status = WaitForCommand(5, FALSE, StopAxis(Index, 
		//				servo_profile[Index].Deceleration, servo_profile[Index].Jerk));
		//if (status.Error == TRUE) {
		//	RtPrintf("StopAxis Error!! Error ID:0x%x\n",status.ErrorId);
		//}

		////The axis is now locked. The state is axisLocked.
		//AxisState state = axisOffline;
		//KsError nRet = GetAxisState(Index, &state);
		//if (nRet != errNoError) {
		//	RtPrintf("Get Axis status Error!!\n");
		//	return -1;
		//}

		//RtPrintf("Current State: %d\n", state);
		////Unlock the axis
		//status = AbortCommand(status); // Cnacel the command
	}
	return 0;
}

int RT_Robot_Controller::QuickStopRobot(void) {
	int ret = 0;
	KsCommandStatus status;
	for (int Index = 0; Index < 6; ++Index) {
		//Stop the axis
		status = StopAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk);
		//status = WaitForCommand(5, FALSE, StopAxis(Index, servo_profile[Index].Deceleration, servo_profile[Index].Jerk));
		if (status.Error) {
			RtPrintf("HaltAxis Error ErrorID:0x%x\n", status.ErrorId);
			return -1;
		}

	}
	return ret;
}

int RT_Robot_Controller::ResetRobotAxis(void) {
	//Makes the transition from the state ErrorStop to Standstill or Disabled by resetting all internal axis-related errors 
	// — it doesn't affect the output of the function instances.
	// When an error occurs on the axis, it sends an alarm and enters the ErrorStop state. This function can clear the alarm.
	KsCommandStatus status;
	for (auto Index = 0; Index < 6; Index++) {
		status = WaitForCommand(5, TRUE, ResetAxis(Index));
		if (status.Error) {
			RtPrintf("Reset Joint %d Error: 0x%x\n", Index, status.ErrorId);
			return -1;
		}
		status = WaitForCommand(5, TRUE, PowerAxis(Index, TRUE, TRUE, TRUE));
	}
	return 0;

}

int RT_Robot_Controller::ResetRobotAlarm(void) {
	/*
	* 目前只進行軟體的 ALARM RESET ，但根據Sanyo的手冊說明，當觸發了 STO 的緊急停止狀態持續 8 秒後 (單迴路的STO EMG)，故障排除後的 ALARM RESET 需要將驅動器的控制電源斷電重開機，
	* 而現在是規劃用EtherCAT的 Remote I/O去控制一組獨立的Relay來將伺服驅動器的控制電源 L1C-L2C 做 啟/斷 ，  但此方案需要修改 HRSS 機箱的部分線路，故暫時還未完成。
	*/

	RtPrintf("ResetRobotAlarm: Not Implemented Yet...\n");
	return 0;
}

unsigned short RT_Robot_Controller::GetRobotErrorState(void) {
	m_error_state = 0;
	// check over-traveling:
	m_error_state |= rt605.isOverTraveled(feedback[0].q.data());
	// check singularities:
	m_error_state |= rt605.isSingular(feedback[0].q.data());

	return m_error_state;
}

int RT_Robot_Controller::GoHome(double _goHome_speed) {
	// _goHome_speed: 六軸回到原點的轉速(單位為 rad/sec.)
	int ret = 0;
	//std::array<double, 6> q_home = { deg2rad(90.0), deg2rad(90.0), deg2rad(0.0), deg2rad(0.0) , deg2rad(-90.0) , deg2rad(0.0) };	
	
	//Start Homing
	for (int i = 0; i < 6; i++) {
		this->OV_Speed = abs(_goHome_speed * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU);

		MoveJointAbs(rt605.kinePrt.joint_home_position.data());
		//status = HomeAxis(Index, rt605.kinePrt.joint_home_position.at(Index),
		//	_goHome_speed, _goHome_speed/10,
		//	servo_profile[Index].Acceleration, servo_profile[Index].Deceleration, servo_profile[Index].Jerk,
		//	mcPositiveDirection, homingLatch);
		///*status = WaitForCommand(30, TRUE, HomeAxis(Index, rt605.kinePrt.joint_home_position.at(Index),
		//	servo_profile[Index].Acceleration/10, servo_profile[Index].Acceleration / 100,
		//	servo_profile[Index].Acceleration, servo_profile[Index].Deceleration, servo_profile[Index].Jerk,
		//	mcPositiveDirection, homingLatch));*/
		//if (status.Error) {
		//	RtPrintf("HomeAxis Error ErrorID:0x%x\n", status.ErrorId);
		//	return -1;
		//}
	}
	return ret;
}

int RT_Robot_Controller::ConfigJogProfile(JogProfileSetting& _setting) {
	// _setting: 設定 JOG模式的加速度等運動資訊。
	/*  example:
				JogProfileSetting setting;
				setting.JointAcceleration_time = 500; // acc time = 500 ms
				setting.JointSpeed = 0.35;			  // JOG SPEED = 0.35 rad/second
				ConfigJogProfile(setting);
*/
	int ret = 0;
	KsError nRet;
	

	for (int i = 0; i < 6; ++i) {
		//SetServoMotionProfileType(i, profileDelayInSecond);
		
		servo_profile[i].MinimumFollowingError = 3;
		servo_profile[i].MaximumFollowingError = 36000; // follow the kingstar motion example
		servo_profile[i].MaximumVelocity = abs(_setting.MaxJointSpeed * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU);
		//if (i >= 2)
		//	servo_profile[i].MaximumVelocity = _setting.MaxJointSpeed * -1 * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		//else
		//	servo_profile[i].MaximumVelocity = _setting.MaxJointSpeed * rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
		char str[50] = { 0 };
		sprintf(str, "%f", servo_profile[i].MaximumVelocity);
		RtPrintf("servo_profile[%d].MaximumVelocity: %s", i, str);
		servo_profile[i].Acceleration = _setting.JointAcceleration_time / 1000.0; // 1 second
		servo_profile[i].Deceleration = _setting.JointAcceleration_time / 1000.0; // 1 second
		servo_profile[i].Jerk = _setting.JointAcceleration_time / 1000.0 * 0.1; 
		servo_profile[i].Jolt = 0.0;
		nRet = SetAxisMotionProfile(i, McProfileType::profileDelayInSecond, servo_profile[i]);
		if (nRet != errNoError) {
			RtPrintf("SetAxisMotionProfile Error: 0x%x\n", nRet);
			return -1;
			
		}
	}
	return ret;
}

void RT_Robot_Controller::select_RobotPriorPosture(int _posture) {
	unsigned short posture = static_cast<unsigned short>(_posture);
	robot_priorPosture = static_cast<Robot_Postures>(posture);
	switch (posture) {
	case 0:
		RtPrintf("MarkUp_ElbowUp_WristFronted\n");
		break;
	case 1:
		RtPrintf("MarkUp_ElbowUp_WristFlipped\n");
		break;
	case 2:
		RtPrintf("MarkUp_ElbowDown_WristFronted\n");
		break;
	case 3:
		RtPrintf("MarkUp_ElbowDown_WristFlipped\n");
		break;
	case 4:
		RtPrintf("MarkDown_ElbowUp_WristFronted\n");
		break;
	case 5:
		RtPrintf("MarkDown_ElbowUp_WristFlipped\n");
		break;
	case 6:
		RtPrintf("MarkDown_ElbowDown_WristFronted\n");
		break;
	case 7:
		RtPrintf("MarkDown_ElbowDown_WristFlipped\n");
		break;
	}
}



/*
* _clk_ms: interrupt period in ms; pRoutine: interrupt service routine
*/
void RT_Robot_Controller::setFeedbackRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context)) {
	// Motion Feedback:
	liPeriod_feedback.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	// Safety Detect:
	// 建立 logger 的執行續，並且處於待命中的狀態：
	hEvent_logger = RtCreateEvent(0, true, false, L"EVENT_MOTION_LOGGER_THREAD");

	hTimer_motion_feedback = RtCreateTimer(NULL, 0, pRoutine, (void*)this, RT_PRIORITY_MAX, CLOCK_FASTEST);
}

void RT_Robot_Controller::setPositionMotionRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context)) {
	liPeriod_motion.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	hTimer_motion = RtCreateTimer(NULL, 0, pRoutine, (void*)this, RT_PRIORITY_MAX - 2, CLOCK_FASTEST);

}

//void RT_Robot_Controller::setPositionForceMotionRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context)) {
//	liPeriod_fc_motion.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
//	hTimer_fc_motion = RtCreateTimer(NULL, 0, pRoutine, (void*)this, RT_PRIORITY_MAX - 2, CLOCK_FASTEST);
//
//}

void RT_Robot_Controller::setSafetyDetectRoutine(double _clk_ms, VOID(RTFCNDCL* pRoutine) (PVOID context)) {
	liPeriod_safety.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	hTimer_motion_safety = RtCreateTimer(NULL, 0, pRoutine, (void*)this, RT_PRIORITY_MAX - 1, CLOCK_FASTEST);

}

void RT_Robot_Controller::StartFeedbackRoutine(void) {
	RtSetTimerRelative(hTimer_motion_feedback, &liPeriod_feedback, &liPeriod_feedback);

}

void RT_Robot_Controller::StartSafetyDetectRoutine(void) {
	RtSetTimerRelative(hTimer_motion_safety, &liPeriod_safety, &liPeriod_safety);

}

void RT_Robot_Controller::StartPositionMotionRoutine(void) {
	RtSetTimerRelative(hTimer_motion, &liPeriod_motion, &liPeriod_motion);
}

void RT_Robot_Controller::UpdateFeedback(void) {
	QueryPerformanceCounter(&feedback[0].current_time_tick);
	feedback[0].time = (double)((feedback[0].current_time_tick.QuadPart - feedback[0].start_time_tick.QuadPart) / feedback[0].cpu_freq);
	for (int i = 0; i < 6; ++i) {
		GetAxisFollowingError(i, McSource::mcActualValue, feedback[0].q_err.data() + i);
		GetAxisPosition(i, McSource::mcSetValue, feedback[0].motor_cmd.data() + i);
		GetAxisPosition(i, McSource::mcActualValue, feedback[0].motor.data() + i);
		GetAxisVelocity(i, McSource::mcActualValue, feedback[0].motor_vel.data() + i);
		GetAxisTorque(i, McSource::mcActualValue, feedback[0].tor.data() + i);

		//GetServoSetPosition(i, feedback[0].motor_cmd.data() + i);
		//GetServoVelocity(i, feedback[0].motor_vel.data() + i);
		//GetServoTorque(i, feedback[0].tor.data() + i);
	}
	//m_fts.ReadData(feedback[0].fts);
	// -----------------------------------------------------------
	// 單位換算：
	// -----------------------------------------------------------
	// 計算關節的角度：(pulse -> rad)
	rt605.CountToJoint(feedback[0].motor.data(), feedback[0].q.data());
	rt605.CountToJoint(feedback[0].motor_cmd.data(), feedback[0].qc.data());
	for (int i = 0; i < 6; ++i)
		feedback[0].dq.at(i) = feedback[0].motor_vel.at(i) / rt605.kinePrt.reduction_ratio[i] * rt605.kinePrt.PPU;
	// 順向運動學計算，計算卡氏座標下的位置向量：
	// Robot Frame 位置向量：
	rt605.RT605_FK_XYZ(feedback[0].q.data(), feedback[0].X0.data());
	rt605.RT605_FK_Orientation(feedback[0].q.data(), feedback[0].X0.data() + 3);
	// Robot Frame 位置命令向量：
	rt605.RT605_FK_XYZ(feedback[0].qc.data(), feedback[0].Xr.data());
	rt605.RT605_FK_Orientation(feedback[0].qc.data(), feedback[0].Xr.data() + 3);
	// Tool Frame 位置向量：
	rt605.RT605_FK_Tcp(feedback[0].q.data(), feedback[0].X.data());
	// -----------------------------------------------------------
}

void RT_Robot_Controller::LogOn(void) {
	feedback[0].RestLogTime();
	errno_t err;
	SYSTEMTIME LocalTime; GetLocalTime(&LocalTime); // generate file name
	char fileName[MAX_PATH] = {};
	char open_file[MAX_PATH];
	char dir[MAX_PATH] = { 0 };
	strcpy(open_file, this->app_dir);

	
	int n = sprintf_s(fileName, sizeof(fileName), "intpLog\\%d_%d_%d_%d_%d_%d.csv", LocalTime.wYear,
		LocalTime.wMonth,
		LocalTime.wDay,
		LocalTime.wHour,
		LocalTime.wMinute,
		LocalTime.wSecond);
	strcat(open_file, fileName);
	err = fopen_s(&logger_fs, open_file, "w+");
	if (err != 0) {
		RtPrintf("Open file %s Error\n", open_file);
		return;
	}
	else {
		fprintf(logger_fs, "time,Xr,Yr,Zr,Rxr,Ryr,Rzr,qc1,qc2,qc3,qc4,qc5,qc6,q1,q2,q3,q4,q5,q6,tor1,tor2,tor3,tor4,tor5,tor6,Fc_x,Fc_y,Fc_z,Fc_tx,Fc_ty,Fc_tz,Fts-Fx,Fts-Fy,Fts-Fz,Fts-Tx,Fts-Ty,Fts-Tz,Ftcp-Fx,Ftcp-Fy,Ftcp-Fz,Ftcp-Tx,Ftcp-Ty,Ftcp-Tz,fc_op1,fc_op2,fc_op3,fc_op4,fc_op5,fc_op6,fc_gain1,fc_gain2,fc_gain3,fc_gain4,fc_gain5,fc_gain6\n");
	}
	RtPrintf("Start logger at %s\n", open_file);
	RtSleep(10);
	this->logger_on = true;
}

void RT_Robot_Controller::LogOn(const char *task, LogMODE log_mode) {
	feedback[0].RestLogTime();
	errno_t err;
	SYSTEMTIME LocalTime; GetLocalTime(&LocalTime); // generate file name
	char date[MAX_PATH] = {};
	char filename[MAX_PATH] = {};

	memset(this->save_file_path, 0, sizeof(char) * MAX_PATH);
	strcpy(this->save_file_path, this->app_dir);
	
	if (log_mode == LogMODE::sweep || log_mode == LogMODE::sinusoidal) {
		strcat_s(this->save_file_path, "intpLog\\");
		strcat_s(this->save_file_path, task);
		sprintf_s(date, sizeof(date), "%d_%d_%d_%d_%d_%d.csv", LocalTime.wYear,
			LocalTime.wMonth,
			LocalTime.wDay,
			LocalTime.wHour,
			LocalTime.wMinute,
			LocalTime.wSecond);
		strcat_s(this->save_file_path, date);
		strcpy(filename, this->save_file_path);
	}
	else if (log_mode == LogMODE::sweepALL) {
		strcpy(filename, task);
	}

	err = fopen_s(&logger_fs, filename, "w+");

	if (err != 0) {
		RtPrintf("Open file %s Error\n", filename);
		return;
	}
	else {
		fprintf(logger_fs, "time,Xr,Yr,Zr,Rxr,Ryr,Rzr,qc1,qc2,qc3,qc4,qc5,qc6,q1,q2,q3,q4,q5,q6,tor1,tor2,tor3,tor4,tor5,tor6,Fc_x,Fc_y,Fc_z,Fc_tx,Fc_ty,Fc_tz,Fts-Fx,Fts-Fy,Fts-Fz,Fts-Tx,Fts-Ty,Fts-Tz,Ftcp-Fx,Ftcp-Fy,Ftcp-Fz,Ftcp-Tx,Ftcp-Ty,Ftcp-Tz,fc_op1,fc_op2,fc_op3,fc_op4,fc_op5,fc_op6,fc_gain1,fc_gain2,fc_gain3,fc_gain4,fc_gain5,fc_gain6\n");
	}
	RtPrintf("Start logger at %s\n", filename);
	RtSleep(10);
	this->logger_on = true;
}

void RT_Robot_Controller::LogOff(void) {
	this->logger_on = false;
	if (logger_fs != nullptr)
		fclose(logger_fs);
}


void RT_Robot_Controller::CancelRoutineTimer(void) {
	RtCancelTimer(hTimer_motion, NULL);
	RtCancelTimer(hTimer_motion_feedback, NULL);
	RtCancelTimer(hTimer_motion_safety, NULL);
}

int RT_Robot_Controller::readPID(PID *rt605PID) {
	/* Read Controller parameters from RT605 */
	//PID rt605PID;
	SubsystemStatus Subsystem = { ecatOffline, ecatOffline, ETHERCAT_SLAVE_COUNT, ETHERCAT_IO_COUNT, ETHERCAT_SERVO_COUNT, {ecatOffline}, {ecatOffline}, {axisOffline} };

	KsError Code = GetStatus(&Subsystem, NULL);
	if (Code != errNoError) {
		RtPrintf("Failed to get status: 0x%x\n", Code);
		return -1;
	}
	RtPrintf("Subsystem status\n  %i Slaves\n  %i I/Os\n  %i Axes\n\n", Subsystem.SlaveCount, Subsystem.IOCount, Subsystem.AxesCount);
	RtPrintf("EtherCAT Current state: %i\n", Subsystem.State);

	unsigned char gain[2] = {0};
	KsCommandStatus Command;
	for (uint16_t i = 0; i < 6; i++) {
		
		Command = ReadAxisSdoObject(i, 0x2005, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Read joint %d kpp Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		rt605PID->kpp[i] = ((uint16_t)gain[0] << 8) | gain[1];
		RtPrintf("joint %i kpp: 0x%i, 0x%i\n", i, gain[0], gain[1]);
		//RtPrintf("joint %i kpp: %i\n",i, rt605PID->kpp[i]);

		Command = ReadAxisSdoObject(i, 0x2006, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Read joint %d kpi Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		rt605PID->kpi[i] = ((uint16_t)gain[0] << 8) | gain[1];
		RtPrintf("joint %i kpi: 0x%04x, 0x%04x\n", i, gain[0], gain[1]);
		//RtPrintf("joint %i kpi: %i\n", i, rt605PID->kpi[i]);
		Command = ReadAxisSdoObject(i, 0x200b, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Read joint %d kvp Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		rt605PID->kvp[i] = ((uint16_t)gain[0] << 8) | gain[1];
		RtPrintf("joint %i kvp: 0x%04x, 0x%04x\n", i, gain[0], gain[1]);
		//RtPrintf("joint %i kvp: %i\n", i, rt605PID->kvp[i]);
		Command = ReadAxisSdoObject(i, 0x200c, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Read joint %d kvi Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		rt605PID->kvi[i] = ((uint16_t)gain[0] << 8) | gain[1];
		RtPrintf("joint %i kvi: 0x%04x, 0x%04x\n", i, gain[0], gain[1]);
		//RtPrintf("joint %i kvi: %i\n", i, rt605PID->kvi[i]);
	}

	return 0;
}

int RT_Robot_Controller::writePID(PID rt605PID) {
	
	unsigned char gain[2] = { 0 };
	KsCommandStatus Command;
	for (unsigned int i = 0; i < 6; i++) {
		gain[0] = (rt605PID.kpp[i] >> 8) & 0xFF; // High byte
		gain[1] = (rt605PID.kpp[i] & 0xFF);	     // Low byte
		Command = WriteAxisSdoObject(i, 0x2005, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Write joint %d PID Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		gain[0] = (rt605PID.kpi[i] >> 8) & 0xFF; // High byte
		gain[1] = (rt605PID.kpi[i] & 0xFF);	     // Low byte
		Command = WriteAxisSdoObject(i, 0x2006, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Write joint %d PID Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		gain[0] = (rt605PID.kvp[i] >> 8) & 0xFF; // High byte
		gain[1] = (rt605PID.kvp[i] & 0xFF);	     // Low byte
		Command = WriteAxisSdoObject(i, 0x200b, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Write joint %d PID Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
		gain[0] = (rt605PID.kvi[i] >> 8) & 0xFF; // High byte
		gain[1] = (rt605PID.kvi[i] & 0xFF);	     // Low byte
		Command = WriteAxisSdoObject(i, 0x200c, 0x01, FALSE, gain, 2);
		if (Command.Error) {
			RtPrintf("Write joint %d PID Error: 0x%x\n", i, Command.ErrorId);
			return Command.ErrorId;
		}
	}



	// check status of servo before set PID
	//for (int i = 0; i < 6; i++) {
	//	if (system_state->servo_status.at(i).PowerOn) {
	//		RtPrintf("Servo %d Power On, please turn the power off before set PID\n", i);
	//		return -1;
	//	}
	//}

	//McPidSettings velPID = {
	//		0,	      //KP
	//		0,   		//KI
	//		0,        //KI_LIMIT_PERCENT
	//		10,       //KD
	//		0.001,    //KV
	//		0.0003,   //KAA
	//		0.0003,   //KAD
	//		0,        //KJ
	//		0.2,      //REDUCED_GAIN_DELAY
	//		0.1,      //REDUCED_GAIN_FACTOR
	//		TRUE,     //KI_STOPPED_ONLY
	//		FALSE,    //KD_USE_INTERNAL_ENCODER
	//		0,        //MINIMUM_OUTPUT
	//		95        //MAXIMUM_OUTPUT
	//};;

	//McPidSettings torPID = {
	//0,	      //KP
	//0,   		//KI
	//0,        //KI_LIMIT_PERCENT
	//10,       //KD
	//0.001,    //KV
	//0.0003,   //KAA
	//0.0003,   //KAD
	//0,        //KJ
	//0.2,      //REDUCED_GAIN_DELAY
	//0.1,      //REDUCED_GAIN_FACTOR
	//TRUE,     //KI_STOPPED_ONLY
	//FALSE,    //KD_USE_INTERNAL_ENCODER
	//0,        //MINIMUM_OUTPUT
	//95        //MAXIMUM_OUTPUT
	//};
	//KsError nRet;
	//for (int i = 0; i < 6; i++) {
	//	// set Kp, need to set control mode to modePidVel to set PID
	//	SetAxisControlMode(i, McControlMode::modePidVel);
	//	velPID.KP = rt605PID[i].kpp;
	//	velPID.KI = rt605PID[i].kpi;
	//	nRet = SetAxisVelocityPid(0, velPID);
	//	if (nRet != 0) {
	//		RtPrintf("Set Velocity mode PID error: %d\n", nRet);
	//		return nRet;
	//	}

	//	SetAxisControlMode(i, McControlMode::modePidTor);
	//	torPID.KP = rt605PID[i].kvp;
	//	torPID.KI = rt605PID[i].kvi;
	//	nRet = SetAxisTorquePid(0, torPID);
	//	if (nRet != 0) {
	//		RtPrintf("Set Torque mode PID error: %d\n", nRet);
	//		return nRet;
	//	}
	//}

	return 0;
}