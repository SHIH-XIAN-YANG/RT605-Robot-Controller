#pragma once
#include<Windows.h>
#include<tchar.h>
#include<string.h>
#include<stdio.h>
#include<cmath>

#include <rtapi.h>    
#ifdef UNDER_RTSS
#include <rtssapi.h> 
#endif // UNDER_RTSS
//#include"ksm64shared.h"
#include "ksapi.h"
#include "ksmotion.h"
#include"../../utility/utility_math.h"

#define PPU_DEG 364.0888888889

enum class SignalType:unsigned short {
	linear=1,
	gaussian=2,
	cos=3,
	sin=4
};

struct FreqSweep {
	SignalType time_signal_type;
	SignalType freq_sweep_type;
	double f0;
	double f1;
	double ts;
	double tf;
	double arg;
	double sweep_slope;
	double sweep_bias;
	int joint_index;
	double q0[6];
};
 
namespace icalab {
	class MechanismAnalyzer
	{
	public:
		LARGE_INTEGER liPeriod_freq_sweep;
		HANDLE hTimer_freq_sweep;
		bool rt_thread_started_ack;
		bool rt_thread_running;

		double* intp;
		unsigned int intp_length;
		unsigned int routine_index;
		struct FreqSweep frequency_sweep;
		//
		MechanismAnalyzer(void);
		~MechanismAnalyzer(void);
		void Initial(void);
		void GenerateSignal(void);
		void Setup(FreqSweep prt);
		void setFrequencySweepISR(double _clk_ms);
		double Linear(double t, double slope, double bias);
		double Gaussian(double t, double std, double var);
		void ConvertToAbsJointPos(double pulse_init);
		//
		void StartFrequencySweep(void);
		
	};
}
void RTFCNDCL RtFrequencySweepRoutine(void* _context);
