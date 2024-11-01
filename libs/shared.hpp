#ifndef __SHARED_MEMORY_2_H__
#define __SHARED_MEMORY_2_H__

#include<stdlib.h>
#include<string.h>
#include<cstddef> 
#include<cstdint>
#include <windows.h>
#include <tchar.h>
//#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.
#include <Rtapi.h>
#include<queue>
#include<array>
//#include<utility>
#include<vector>

#include "../libs/icalab/rc/ParameterEditor/ParameterEditor.h"
#include"../libs/icalab/RT605/rt605Kinematics.h"
#include"./icalab/rc/peripheral/peripheral.h"

#define UI_MAILBOX_NAME L"UI_MAILBOX_Name"
#define UI_EVENT_NAME L"UI_ACCESS_EVENT"
#define UI_ACK_EVENT_NAME L"UI_ACK_EVENT"
#define SHM_MOTIONFEEDBACK_NAME L"SHM_MOTION_FEEDBACK"
#define SHM_SYSTEM_STATE L"SHM_SYSTEM_STATE"

#define SHM_MOTIONFEEDBACK_NAME L"SHM_MOTION_FEEDBACK"
#define SHM_SYSTEM_STATE L"SHM_SYSTEM_STATE"
#define SHM_PID_NAME  L"SHM_PID"
#define SHM_RTSS_STATE_NAME  L"SHM_RTSS_TASK_STATUS"

#define WRITE_CMD(cmd) static_cast<void*>(&cmd)

enum class LogFileAction : int {
	NOFILECREATED,
	CreateFileRequest,
	CreateFileFailed,
	CloseFileRequest,
	CloseFileFailed,
	FileLogging
};

enum class RT605Action : int {
	SINUSOIDAL_TEST,
	FREQ_SWEEP,
	TASK_HOLD,
	TASK_RESUME,
	TASK_ABORT,
	IDEL
};

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS


// define RTX real-time state
#define KSM_INITIALIZING 0
#define KSM_INITIALZED 1
#define RT_READY2WRITE_DATA 2
#define RT_RUNNING 3
#define RT_TERMINATING 4


struct PsdData {
	double psd1_vx1, psd1_vx2, psd1_vy1, psd1_vy2;
	double psd2_vx1, psd2_vx2, psd2_vy1, psd2_vy2;
	struct Shm_UI* ui;
};
#pragma region typedefine_and_enumerate
enum class UI_Action : int { // UI 8-bit Action
	None = 0,
	Rtss_Ack = -1,
	//----- Group-0: ------
	TerminateRtProcess = 1,
	Disconnect = 2,
	Connect = 3,
	ImportRobotPrt = 4,
	ServoON = 5,
	ServoOFF = 6,
	Stop = 7,
	Resume = 8,
	QuickStop = 9,
	LogON = 10,
	LogOFF = 11,
	LogAuto = 12,
	ShowPulse = 13,
	GoHome = 14,
	AlarmReset = 15,
	RobotAxisReset = 16,
	HaltAll,
	//----- Group-1: ------
	Apply1 = 101,
	JogStop = 102,
	Jog_q = 103,
	Jog_c = 104,
	MovePtpXYZ = 105,
	MovePtpOrientation = 106,
	MovePtpToolXYZ = 107,
	MovePtpToolOrientation = 108,
	MoveLineXYZ = 109,
	MoveLineToolXYZ = 110,
	MoveJointAbs = 111,
	MoveJointRel = 112,
	ConfigJogProfile = 113,
	//----- Group-2: ------
	LoadTask = 201,
	StartINTPtask = 202,
	INTPtaskEnd = 203,
	//----- Group-3: ------
	//Reserved = 301,
	CalibrateTcpFrame = 302,
	CalibrateWorkspaceFrame = 303,
	//----- Group-4: (Servo Tuning) 
	SetFrequencySweep = 401,
	StartFrequencySweep = 402,
	StartFrequencySweepALL,
	ImportFFTresult,
	RequestForReadServoPID,
	WriteServoPID,
	SetupSinusoidalTest,
	StartSinusoidalTest,
	// ----- Group-10: (From Rtss to Win)
	ReturnRequestedServoPID,
};

enum class PID_Type {
	Kpp,
	Kpi,
	Kvp,
	Kvi
};

typedef struct PID {
	bool busy;
	UINT16 kpp[6];
	UINT16 kpi[6];
	UINT16 kvp[6];
	UINT16 kvi[6];
} PID;

typedef struct RTSSTask_Status {
	char file_name[1024];
	bool busy;
} RTSSTask_Status;

//enum class 	ServoControlMode : unsigned char {
//	None,
//	pp,		// Profile Position 
//	csp,	// Cyclic Synchronous Position
//	pv,		// Velocity
//	csv,
//	pt,		// Torque
//	cst
//};
//
enum class Node : unsigned short {
	None = 0,
	Win_HMI = 1,
	RTSS_main = 2
};
//

enum class DataType : unsigned char {
	None = 0,
	Bit,	// (101) BOOL
	I8,		// (108) char
	I16, 	// (116) short
	I32, 	// (132) int
	I64,    // (164) long long
	U8,  	// (208) unsigned char
	U16, 	// (216) unsigned short
	U32, 	// (232) unsigned int
	U64,    // (264) unsigned long long
	F32, 	// (332) float
	F64, 	// (364) double

	PathString,
	Boolarray288,
	F64array36,
	F32array66,
	I64array36,
	I32array66,
	U64array36,
	U32array66,
	I8Parray288,
	U8array288,
	I16array144,
	U16array144
};

typedef struct MailBox {
	/*
	  - 整個 MailBox 物件記憶體大小為 256 bits (32 bytes)，其中由於 成員 "content<MailContent>" 大小就有 192 bits，
		而在 記憶體對齊的機制下 MailBox 類別會將其餘的 成員屬性 封裝至另一組 64 bits 的空間，故整體 MailBox 類別實例化後會占用 64*2 = 128 bits 的記憶體大小，這也是 IPC 需要共享的記憶體空間。

	  - 由於此設計是基於 shared memory 之 IPC 模式，所以會有 race condition 存在，
		為了避免 Data racing，故在讀寫 MailBox 的內容時要使用 "WriteMessage" 與 "ReadMessage" 等方法來達到 非同步 的 IPC，非必要盡量避免直接 access 成員變數。
		(p.s. 張晉蒝 大哥有建議可以嘗試虛擬 TCP/IP 的方式來進行真正的 mail-box 之 IPC 模式，可以直接避免 race condition 發生，同時也不用再自行設計 "非同步" 的機制，日後有機會可以使用 RTX 的 RT-TCP/IP 來嘗試看看)
	*/

	// 要傳遞的參數與訊息: 
	union {
		// 為了控制所有不同型態的 參數 之 記憶體大小，以節省記憶體 mapping 的負擔：
		// arg:
		bool               dataBool;
		char               dataI8;
		short              dataI16;
		int                dataI32;
		long long          dataI64;
		unsigned char      dataU8;
		unsigned short     dataU16;
		unsigned int       dataU32;
		unsigned long long dataU64;
		float              dataF32;
		double             dataF64;
		// arg array:
		char			   pathStr[MAX_PATH];
		float			   F32array66[66];
		double			   F64array36[36];
		bool			   Boolarray288[288];
		char			   I8array288[288];
		unsigned char	   U8array288[288];
		short			   I16array144[144];
		unsigned short	   U16array144[144];
		int                I32array66[66];
		unsigned int       U32array66[66];
		unsigned long long U64array36[36];
		long long		   I64array36[36];
	};

	bool Connected;
	bool Ack;
	bool Busy; //  True:busy / False:idle
	
	//unsigned int Length;
	Node Sender;
	Node Receiver;
	UI_Action Action;
	DataType data_type;
	// Constructor:
	void Initialize(void) {
		this->Connected = false;
		this->Ack = false;
		this->Busy = false;
		//this->Length = 1;
		this->Empty_Mailbox();
	}
	//
	inline void setConnected(HANDLE hAccess_Event, bool a) { // for WinGUI
		RtWaitForSingleObject(hAccess_Event, INFINITE);
		RtResetEvent(hAccess_Event);
		Busy = true;
		if (a == true) {
			Connected = true;
		}
		else if (a == false) {
			Connected = false;
		}
		Ack = true;
		Busy = false;
		RtSetEvent(hAccess_Event);
	}
	inline bool isConnected(HANDLE hAccess_Event) { // for RTSS
		bool ret{ false };
		RtWaitForSingleObject(hAccess_Event, INFINITE);
		RtResetEvent(hAccess_Event);
		Busy = true;
		Connected == true ? ret = true : false;
		Busy = false;
		RtSetEvent(hAccess_Event);
		return ret;
	}

	// Access Method:
	bool WriteMessage(HANDLE hAccess_Event,
		Node sender,
		Node receiver,
		UI_Action action,
		unsigned short len,
		DataType dtype,
		void* PDATA)
	{
		bool nRet;
		RtWaitForSingleObject(hAccess_Event, INFINITE);
		RtResetEvent(hAccess_Event);
		if (!Busy) {
			Busy = true;
			Sender = sender;
			Receiver = receiver;
			data_type = dtype;
			Action = action;
			// Cast the data type:
			switch (dtype) {
			case DataType::Bit: { // bit
				dataBool = *(static_cast<bool*>(PDATA));
				break;
			}
			case DataType::I8: {
				dataI8 = *(static_cast<char*>(PDATA));
				break;
			}
			case DataType::I16: {
				dataI16 = *(static_cast<short*>(PDATA));
				break;
			}
			case DataType::I32: {
				dataI32 = *(static_cast<int*>(PDATA));
				break;
			}
							  //case DataType::I64: {
							  //	dataI64 = *(static_cast<long long*>(PDATA));
							  //	break;
							  //}
			case DataType::U8: {
				dataU8 = *(static_cast<unsigned char*>(PDATA));
				break;
			}
			case DataType::U16: {
				dataU16 = *(static_cast<unsigned short*>(PDATA));
				break;
			}
			case DataType::U32: {
				dataU32 = *(static_cast<unsigned int*>(PDATA));
				break;
			}
							  //case DataType::U64: {
							  //	dataU64 = *(static_cast<unsigned long long*>(PDATA));
							  //	break;
							  //}
			case DataType::F32: {
				dataF32 = *(static_cast<float*>(PDATA));
				break;
			}
			case DataType::F64: {
				dataF64 = *(static_cast<double*>(PDATA));
				break;
			}
			case DataType::F32array66: {
				for (int i = 0; i < len; ++i)
					F32array66[i] = *(static_cast<float*>(PDATA) + i);
				break;
			}
			case DataType::F64array36: {
				for (int i = 0; i < len; ++i) {
					F64array36[i] = *(static_cast<double*>(PDATA) + i);
				}
				break;
			}
			case DataType::Boolarray288: {
				for (int i = 0; i < len; ++i)
					Boolarray288[i] = *(static_cast<bool*>(PDATA) + i);
				break;
			}
			case DataType::I8Parray288: {
				for (int i = 0; i < len; ++i)
					I8array288[i] = *(static_cast<char*>(PDATA) + i);
				break;
			}
			case DataType::U8array288: {
				for (int i = 0; i < len; ++i)
					U8array288[i] = *(static_cast<unsigned char*>(PDATA) + i);
				break;
			}
			case DataType::I16array144: {
				for (int i = 0; i < len; ++i)
					I16array144[i] = *(static_cast<short*>(PDATA) + i);
				break;
			}
			case DataType::U16array144: {
				for (int i = 0; i < len; ++i)
					U16array144[i] = *(static_cast<unsigned short*>(PDATA) + i);
				break;
			}
			case DataType::I32array66: {
				for (int i = 0; i < len; ++i)
					I32array66[i] = *(static_cast<int*>(PDATA) + i);
				break;
			}
			case DataType::U32array66: {
				for (int i = 0; i < len; ++i)
					U32array66[i] = *(static_cast<unsigned int*>(PDATA) + i);
				break;
			}
			case DataType::I64array36: {
				for (int i = 0; i < len; ++i)
					I64array36[i] = *(static_cast<unsigned long long*>(PDATA) + i);
				break;
			}
			case DataType::U64array36: {
				for (int i = 0; i < len; ++i)
					U64array36[i] = *(static_cast<long long*>(PDATA) + i);
				break;
			}
			case DataType::PathString: {
				strcpy_s(pathStr, static_cast<char*>(PDATA));
				break;
			}
									 //case DataType::PrtF64 : {
									 //	for(int i = 0; i<14; ++i){
									 //		prtF64.at(i) = *((static_cast<double*>(PDATA))+i);
									 //	}
									 //break;
									 //}
													   //case DataType::String: {
													   //	memcpy(dataStr, PDATA, 127);
													   //	break;
													   //}
			}
			nRet = true;
		}
		else if (Busy == true) {
			nRet = false;
		}
		// Return code:
		Ack = true;
		Busy = false;
		RtSetEvent(hAccess_Event);
		return nRet;
	}
	//
	void Empty_Mailbox(void) {
		this->Sender = Node::None;
		this->Receiver = Node::None;
		dataF64 = 0.0;
	}
	//
	bool ReadMessage(HANDLE hAccess_Event, MailBox& receiveBuf) {
		bool nRet{ false };
		RtWaitForSingleObject(hAccess_Event, INFINITE);
		RtResetEvent(hAccess_Event);
		if (!Busy) {
			Busy = true;
			receiveBuf = std::move(*this);
			nRet = true;
		}
		// Return code:
		Ack = false;
		Empty_Mailbox();
		RtSetEvent(hAccess_Event);
		return nRet;
	}
} MailBox, * pMailBox;

#pragma endregion typedfine_and_enumerate
struct Shm_UI {
	bool run;
	bool log;
	int cmd;
	double x1, y1, sigma1;
	double x2, y2, sigma2;
	bool IRED_EN;
	bool IRED_prev_state;

	int rt_state;
	double robot_pos[6] = { 0 };
	double robot_ang[6] = { 0 };
	char record_filename[256] = { 0 };
	LogFileAction file_create;
	RT605Action rt605_action;
	FILE* fs;

	char app_dir[256];

	std::vector<std::vector<double>> joints_err;
	std::vector<double> joint_data_act;
	std::vector<double> joint_data_ref;

	UI_Action ui_action;

	Shm_UI(void) {
		cmd = 0;
		log = false;
		run = false;
		rt605_action = RT605Action::IDEL;
		file_create = LogFileAction::NOFILECREATED;
		ui_action = UI_Action::None;
		x1 = y1 = sigma1 = 0;
		x2 = y2 = sigma2 = 0;
		IRED_EN = true;
		IRED_prev_state = true;
		rt_state = 0;
		fs = nullptr;
	}
};



#pragma region Motion State
typedef struct MotionFeedback {
	icalab::RT605<double>* pRt605;

	// 取得目前 CPU frequency
	double cpu_freq;
	LARGE_INTEGER start_time_tick;
	LARGE_INTEGER current_time_tick;
	double time; // unist: ms
	// Servo Drive:
	std::array<double, 7> Xr;   // m and rad
	std::array<double, 7> X;   // m and rad
	std::array<double, 7> X0;   // m and rad
	std::array<double, 6> qc; // rad
	std::array<double, 6> q; // rad
	std::array<double, 6> q_err; // N-m
	std::array<double, 6> dq;
	std::array<double, 6> motor_cmd;  // pulse
	std::array<double, 6> motor;   // pulse 
	std::array<double, 6> motor_vel;  // pulse/sec.
	std::array<double, 6> tor; // N-m

	//
	MotionFeedback(void) {

	}
	//
	void set_RobotModel(icalab::RT605<double>* _robot) {
		pRt605 = _robot;
	}
	//
	void RestLogTime(void) {
		LARGE_INTEGER tmp;
		QueryPerformanceFrequency(&tmp);
		cpu_freq = double(tmp.QuadPart) / 1000.0; // compute the frequency of system clock (unit: 1ms)
		QueryPerformanceCounter(&start_time_tick);
	}
	//

} *pMotionFeedback, MotionFeedback;
#pragma endregion Motion State
//
#pragma region real-time-log
class RtLogger {
private:
	FILE* fs_m;
	char file_name_m[100];
	bool m_started;

	float time; // (1) (unit: ms) 
	std::array<double, 6> Xr;	// (2~7)   (unit: mm, degree)
	std::array<double, 6> qc;	// (8~13)  (unit: degree)
	std::array<double, 6> q;		// (14~19) (unit: pulse)
	std::array<double, 6> tor;	// (20~25) (unit: 0.1 % rated)
	std::array<unsigned char, 8> system_state; // (26) (unit: byte)
	std::array<unsigned char, 8> ctrl_state;   // (27) (unit: byte)
	struct Custom_A {
		std::array<unsigned char, 8> ctrl_state; // (28) (unit: byte)
		std::array<float, 6> FTS; // (29~34) (unit: N, N-m)	
	} custom_A;
	struct CustomLog_Bool {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		bool state;
	};
	struct CustomLog_U8 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		unsigned char state;
	};
	struct CustomLog_U16 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		unsigned short state;
	};
	struct CustomLog_U32 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		unsigned int state;
	};
	struct CustomLog_I8 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		char state;
	};
	struct CustomLog_I16 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		short state;
	};
	struct CustomLog_I32 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		int state;
	};
	struct CustomLog_F32 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		float state;
	};
	struct CustomLog_F64 {
		char name[32];
		//std::array<unsigned char, 8> ctrl_state;
		double state;
	};

public:
	unsigned int custom_data;
	std::vector<CustomLog_Bool>	custom_bool_vec;
	std::vector<CustomLog_I8>	custom_I8_vec;
	std::vector<CustomLog_I16>	custom_I16_vec;
	std::vector<CustomLog_I32>	custom_I32_vec;
	std::vector<CustomLog_U8>	custom_U8_vec;
	std::vector<CustomLog_U16>	custom_U16_vec;
	std::vector<CustomLog_U32>	custom_U32_vec;
	std::vector<CustomLog_F32>	custom_F32_vec;
	std::vector<CustomLog_F64>	custom_F64_vec;

	inline void AddItem(DataType type, const char* name) {
		switch (type) {
		case DataType::Bit: {
			size_t size = this->custom_bool_vec.size();
			this->custom_bool_vec.reserve(size + 1);
			this->custom_bool_vec.resize(size + 1);
			memcpy(custom_bool_vec.at(size).name, name, strlen(name));
		}
		case DataType::I8: {
			size_t size = this->custom_I8_vec.size();
			this->custom_I8_vec.reserve(size + 1);
			this->custom_I8_vec.resize(size + 1);
			memcpy(custom_I8_vec.at(size).name, name, strlen(name));
		}
		case DataType::I16: {
			size_t size = this->custom_I16_vec.size();
			this->custom_I16_vec.reserve(size + 1);
			this->custom_I16_vec.resize(size + 1);
			memcpy(custom_I16_vec.at(size).name, name, strlen(name));
		}
		case DataType::I32: {
			size_t size = this->custom_I32_vec.size();
			this->custom_I32_vec.reserve(size + 1);
			this->custom_I32_vec.resize(size + 1);
			memcpy(custom_I32_vec.at(size).name, name, strlen(name));
		}
		case DataType::U8: {
			size_t size = this->custom_U8_vec.size();
			this->custom_U8_vec.reserve(size + 1);
			this->custom_U8_vec.resize(size + 1);
			memcpy(custom_U8_vec.at(size).name, name, strlen(name));
		}
		case DataType::U16: {
			size_t size = this->custom_U16_vec.size();
			this->custom_U16_vec.reserve(size + 1);
			this->custom_U16_vec.resize(size + 1);
			memcpy(custom_U16_vec.at(size).name, name, strlen(name));
		}
		case DataType::U32: {
			size_t size = this->custom_U32_vec.size();
			this->custom_U32_vec.reserve(size + 1);
			this->custom_U32_vec.resize(size + 1);
			memcpy(custom_U32_vec.at(size).name, name, strlen(name));
		}
		case DataType::F32: {
			size_t size = this->custom_F32_vec.size();
			this->custom_F32_vec.reserve(size + 1);
			this->custom_F32_vec.resize(size + 1);
			memcpy(custom_F32_vec.at(size).name, name, strlen(name));
		}
		case DataType::F64: {
			size_t size = this->custom_F64_vec.size();
			this->custom_F64_vec.reserve(size + 1);
			this->custom_F64_vec.resize(size + 1);
			memcpy(custom_F64_vec.at(size).name, name, strlen(name));
		}
		}

	}
	//
	RtLogger(void) {
		this->fs_m = NULL;
		this->m_started = false;
	}
	//
	inline void WriteInitialState(void) {}
	//
	inline void Update(void) {
	}
	//
	inline void SetFileName(const char* file_name) {
		char dir[] = "d:/ICRC_Log/";
		CreateDirectoryA(dir, NULL);
		memcpy(this->file_name_m, dir, strlen(dir));
		strcat(file_name_m, file_name);
		printf("Set the file directory of real-time log: %s\n", file_name_m);
	}
	//
	inline void Start(void) {
		printf("Starting the real-time logger.\n");
		if (fs_m == nullptr) {
			printf(file_name_m);
			fopen_s(&fs_m, const_cast<char*>(file_name_m), "w+");
			if (fs_m != NULL) {
				printf("The file: '%s' has been opened.\n", file_name_m);
				m_started = true;
			}
		}
		else {
			fclose(fs_m);
			printf("The file has been established, the existed file will be overrided.\n");
			fopen_s(&fs_m, const_cast<char*>(file_name_m), "w+");
			if (fs_m != NULL) {
				printf("The file: '%s' has been opened.\n", file_name_m);
				m_started = true;
			}
		}
		// Add headers:
		fprintf_s(fs_m, "time,Xr,Yr,Zr,Rxr,Ryr,Rzr,qc1,qc2,qc3,qc4,qc5,qc6,q1,q2,q3,q4,q5,q6,tor1,tor2,tor3,tor4,tor5,tor6,system,ctrl-1");
		fprintf_s(fs_m, ",ctrl-A,Fts-Fx,Fts-Fy,Fts-Fz,Fts-Tx,Fts-Ty,Fts-Tz");
		if (custom_bool_vec.size() > 0)
			for (unsigned int i = 0; i < custom_bool_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_bool_vec.at(i).name);
			}
		if (custom_I8_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I8_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_I8_vec.at(i).name);
			}
		if (custom_I16_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I16_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_I16_vec.at(i).name);
			}
		if (custom_I32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_I32_vec.at(i).name);
			}
		if (custom_U8_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U8_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_U8_vec.at(i).name);
			}
		if (custom_U16_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U16_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_U16_vec.at(i).name);
			}
		if (custom_U32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_U32_vec.at(i).name);
			}
		if (custom_F32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_F32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_F32_vec.at(i).name);
			}
		if (custom_F64_vec.size() > 0)
			for (unsigned int i = 0; i < custom_F64_vec.size(); ++i) {
				fprintf_s(fs_m, ",%s", custom_F64_vec.at(i).name);
			}
		fprintf_s(fs_m, "\n");
	}
	inline void Close(void) {
		if (fs_m != nullptr)
			fclose(fs_m);
		m_started = false;
		printf("Close the logger.\n");
	}
	//
	inline void WriteInitialState(KinematicParameter kinematics) {
		fprintf_s(fs_m,
			"%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i",
			0, 0, 0, 0, 0, 0, 0, kinematics.encoder_zero_pulse.at(0), kinematics.encoder_zero_pulse.at(1), kinematics.encoder_zero_pulse.at(2), kinematics.encoder_zero_pulse.at(3), kinematics.encoder_zero_pulse.at(4), kinematics.encoder_zero_pulse.at(5),
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		if (custom_bool_vec.size() > 0)
			for (unsigned int i = 0; i < custom_bool_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_I8_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I8_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_I16_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I16_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_I32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_I32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_U8_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U8_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_U16_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U16_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_U32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_U32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%d", 0);
			}
		if (custom_F32_vec.size() > 0)
			for (unsigned int i = 0; i < custom_F32_vec.size(); ++i) {
				fprintf_s(fs_m, ",%f", 0.0);
			}
		if (custom_F64_vec.size() > 0)
			for (unsigned int i = 0; i < custom_F64_vec.size(); ++i) {
				fprintf_s(fs_m, ",%lf", 0.0);
			}
		fprintf_s(fs_m, "\n");
		printf("The initial state has been writen.\n");
	}
	inline void ExportData(void) {

	}
	//
	inline bool isStarted(void) {
		return m_started;
	}
};

#pragma endregion real-time-log

// System State Monitor:
/*
* includes:
*	(a) robot and joint motion state
*	(b) servo driver CANopen state
*	(c) KING START and EtherCAT Fieldbus status
*/
enum class RcSystemStateAction {
	update,
	copy,
	isIntarget,
	isServoON
};
typedef struct RcSystemState {


	char app_directory[128];
	bool ksm_started;

	std::array<AxisStatus, 6> servo_status;
	std::array<McMotionState, 6> servoMotion_state;
	int ksm_state;
	KsError nRet = errNoError;
	KsCommandStatus Command = { 0 };
	SubsystemStatus Subsystem = { ecatOffline, ecatOffline, 0, 0, 0, {ecatOffline}, {ecatOffline}, {axisOffline} };


	HANDLE m_mutex;
	//
	RcSystemState(void) {
		
		m_mutex = RtCreateMutex(0, false, SHM_SYSTEM_STATE);
	}
	~RcSystemState(void) {
		RtCloseHandle(m_mutex);
	}
	//
	//
	int GetInformation(RcSystemStateAction _action, struct RcSystemState& reg) {
		int ret = 0;
		RtWaitForSingleObject(m_mutex, INFINITE);
		// -------------------------------------------------------------
		if (_action == RcSystemStateAction::copy) { // read:
			memcpy(reg.servo_status.data(), this->servo_status.data(), 6 * sizeof(AxisStatus));
			memcpy(reg.servoMotion_state.data(), this->servoMotion_state.data(), 6 * sizeof(McMotionState));
			reg.ksm_state = this->ksm_state;
		}
		else if (_action == RcSystemStateAction::update) { // write: (update)
			for (int i = 0; i < 6; ++i) {
				int* axis_idx = new int[1];
				axis_idx[0] = i;
				McDirection mcDirection = McDirection::mcPositiveDirection;
				GetAxesStatus(1, axis_idx, servo_status.data() + i);
				//GetServoStatus(i, servo_status.data() + i);
				
				GetAxisMotionState(i, servoMotion_state.data() + i, &mcDirection);
				//GetServoMotionStatus(i, servoMotion_state.data() + i);
			}
			nRet = GetStatus(&Subsystem, NULL);
			//ksm_state = GetLinkState(); // legacy
		}
		else if (_action == RcSystemStateAction::isIntarget) {
			ret = 1;
			for (int i = 0; i < 6; ++i)
				ret &= servo_status.at(i).State;
		}
		else if (_action == RcSystemStateAction::isServoON) {
			ret = 1;
			for (int i = 0; i < 6; ++i) {
				if (servo_status.at(i).State == 3) { //The axis is powered off.
					RtPrintf("Axis %d Servo off\n", i);
					ret = 0;
				}
			}
		}
		// -------------------------------------------------------------
		RtReleaseMutex(m_mutex);
		return ret;
	}
	//
	inline bool isInTarget(void) {
		return static_cast<bool>(GetInformation(RcSystemStateAction::isIntarget, *(this)));
	}
	//
	inline bool isServoON(void) {
		for (int i = 0; i < 6; ++i)
			//RtPrintf("%i,  ", (int)(servo_status.at(i).CiA402State));
		RtPrintf("\n");
		return static_cast<bool>(GetInformation(RcSystemStateAction::isServoON, *(this)));
	}

} *pRcSystemState, RcSystemState;

#endif // SHARED_HPP