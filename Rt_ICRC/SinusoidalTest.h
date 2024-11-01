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

struct Sinusoidal {
	double amplitude;
	double ts;
	double tf;
	int joint_index;
	double tukey_window_alpha;
	double q0[6] = { 0,0,0,0,-90,0 };
	Sinusoidal() {
		joint_index = -1;
		amplitude = 0;
		ts = 0.0; // start time
		tf = 1.0; //final time
		tukey_window_alpha = 0.2;
	}
};

namespace icalab {
	class SinusoidalTest
	{
	public:
		LARGE_INTEGER liPeriod_sineTest;
		HANDLE hTimer_sine_test;

		bool rt_thread_started_ack;
		bool rt_thread_running;

		double** intp;
		char app_dir[256];

		unsigned int intp_length;
		unsigned int routine_index;
		struct Sinusoidal sinusoidal_test_params;
		
		double ratio[6] = { 80.0, 100.0, -80.0, -81.0, -80.0, -50.0 }; //Reduction Ratio

		SinusoidalTest();
		~SinusoidalTest();

		void ConvertToAbsJointPos(double *);

		void generate_Sinusoidal_signal(Sinusoidal);
		void generate_tukey_window(double*, int, double);
		void setSinusoidalTestISR(double _clk_ms);
		void StartSinusoidalTest(void);
		int set_app_dir(char *);
	};
}

void RTFCNDCL RtSinusoidalTestRoutine(void* _context);