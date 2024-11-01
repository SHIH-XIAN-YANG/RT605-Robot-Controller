#pragma once
#include<Windows.h>
#include<tchar.h>
#include<string.h>
#include<stdio.h>
#define _USE_MATH_DEFINES
#include<cmath>

#include <rtapi.h>    
#ifdef UNDER_RTSS
#include <rtssapi.h>
#endif // UNDER_RTSS

#include "ksapi.h"
#include "ksmotion.h"

#define PPU_DEG 364.0888888889

enum class SignalType :unsigned short {
	linear,
	gaussian,
	cos,
	sin
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
	double q0[6] = { 0,0,0,0,-90,0 };
	FreqSweep() {
		arg = 0.1;
		f0 = 0.1; // from 0.1Hz
		f1 = 100; // from 100Hz
		freq_sweep_type = SignalType::sin; // chirp sine
		joint_index = -1;
		sweep_bias = 0.0;
		sweep_slope = 0.0;
		ts = 0.0;
		tf = 1.0;
		time_signal_type = SignalType::linear;
	}
};

//struct Sinusoidal {
//	double amplitude;
//	double ts;
//	double tf;
//	int joint_index;
//	double q0[6] = { 0,0,0,0,-90,0 };
//	Sinusoidal() {
//		joint_index = -1;
//		amplitude = 0;
//		ts=0.0; // start time
//		tf=1.0; //final time
//	}
//};

namespace icalab {
	class MechanismAnalyzer
	{
	public:
		LARGE_INTEGER liPeriod_freq_sweep;
		HANDLE hTimer_freq_sweep;

		//LARGE_INTEGER liPeriod_sineTest;
		//HANDLE hTimer_sine_test;

		bool rt_thread_started_ack;
		bool rt_thread_running;

		double* intp;
		char app_dir[MAX_PATH];


		unsigned int intp_length;
		unsigned int routine_index;
		struct FreqSweep frequency_sweep_params;
		//struct Sinusoidal  sinusoidal_test_params;

		MechanismAnalyzer();
		~MechanismAnalyzer();
		

		void Setup(FreqSweep);

		void generate_INTP_Signal(void);
		void setFrequencySweepISR(double _clk_ms);

		int set_app_dir(char*);


		double Linear(double t, double slope, double bias);
		double Gaussian(double t, double std, double var);
		void ConvertToAbsJointPos(double pulse_init);

		void StartFrequencySweep(void);
	
		
	};
}


void RTFCNDCL RtFrequencySweepRoutine(void* _context);
