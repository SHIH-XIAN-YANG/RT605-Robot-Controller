#include "SinusoidalTest.h"


icalab::SinusoidalTest::SinusoidalTest(void) {
	this->intp = nullptr;
	this->routine_index = 0;
	this->rt_thread_started_ack = false;
	this->rt_thread_running = false;
}

icalab::SinusoidalTest::~SinusoidalTest(void) {
	for (int i = 0; i < 6; i++) {
		free(this->intp[i]);
	}
	free(this->intp);
}

int icalab::SinusoidalTest::set_app_dir(char *app_dir) {
	errno_t err = strcpy_s(this->app_dir, app_dir);
	if (err != 0) {
		RtPrintf("Set app directory error\n");
	}
	return err;
}


#pragma region SinusoidalTest

void icalab::SinusoidalTest::generate_tukey_window(double* window, int data_length, double alpha) {
	int n;
	int taper_length = (int)(alpha * (data_length - 1) / 2.0);

	for (n = 0; n < data_length; n++) {
		if (n < taper_length) {
			window[n] = 0.5 * (1 + cos(M_PI * (2.0 * n / (alpha * (data_length - 1)) - 1.0)));
		}
		else if (n >= data_length - taper_length) {
			window[n] = 0.5 * (1 + cos(M_PI * (2.0 * (n - data_length + 1) / (alpha * (data_length - 1)) + 1.0)));
		}
		else {
			window[n] = 1.0;
		}
	}
}

void icalab::SinusoidalTest::generate_Sinusoidal_signal(Sinusoidal params) {

	memcpy(&this->sinusoidal_test_params, &params, sizeof(Sinusoidal));
	this->intp_length = static_cast<unsigned int>(params.tf / params.ts);
	RtPrintf("INTP length = %d\n", this->intp_length);

	// initialize the INTP buffer:
	if (this->intp == nullptr) {
		this->intp = static_cast<double**>(malloc(sizeof(double*) * 6));
		for (int i = 0; i < 6; i++) {
			this->intp[i] = (double*)malloc(sizeof(double) * this->intp_length);
		}
	}
	else {
		free(this->intp);
		this->intp = static_cast<double**>(malloc(sizeof(double*) * 6));
		for (int i = 0; i < 6; i++) {
			this->intp[i] = (double*)malloc(sizeof(double) * this->intp_length);
		}
	}

	// Setup ISR:
	this->setSinusoidalTestISR(params.ts);

	/* Generate Sine */
	double f = 0.0;
	double time = 0.0;

	errno_t err;
	FILE* fs = nullptr;
	char path[MAX_PATH] = { 0 };
	strcpy(path, app_dir);
	strcat(path, "data_log\\sinusoidal_ref.csv");

	err = fopen_s(&fs, path, "w+");
	if (err != 0) {
		RtPrintf("Error: Could not open file: %s\n", path);
		return;
	}
	fprintf(fs, "time,J1,J2,J3,J4,J5,J6\n");

	double* smooth_sine = (double*)malloc(sizeof(double) * this->intp_length);
	double* window = (double*)malloc(sizeof(double) * this->intp_length);

	generate_tukey_window(window, this->intp_length, this->sinusoidal_test_params.tukey_window_alpha);
	
	// Generate sinusoidal wave with tukey window
	for (int i = 0; i < this->intp_length; i++) {
		time = (double)i * sinusoidal_test_params.ts * 0.001; // unit: second	
		smooth_sine[i] = sinusoidal_test_params.amplitude * sin(2.0 * M_PI * time / (sinusoidal_test_params.tf * 0.001));
		smooth_sine[i] = smooth_sine[i] * window[i];
	}


	for (unsigned int i = 0; i < 6; i++) {
		for (unsigned int j = 0; j < this->intp_length; ++j) {
			this->intp[i][j] = smooth_sine[j];
		}
	}

	// Convert to Abs. Joint Position:
	double *pulse_init = (double *)malloc(sizeof(double)*6);
	KsError nRet;
	for (int i = 0; i < 6; i++) {
		nRet = GetAxisPosition(i, McSource::mcActualValue, &pulse_init[i]); // Not sure if it is right
		if (nRet != errNoError) {
			RtPrintf("Get Axix Position Error!!: %d\n", nRet);
		}
		
		
	}
	this->ConvertToAbsJointPos(pulse_init);
	
	char str[256] = "";
	
	for (int j = 0; j < this->intp_length; j++) {
		time = (double)j * sinusoidal_test_params.ts * 0.001; // unit: second
		fprintf(fs, "%f, %f, %f, %f, %f, %f, %f\n", time, intp[0][j], intp[1][j], intp[2][j], intp[3][j], intp[4][j], intp[5][j]);
		//for (int i = 0; i < 6; i++) {
		//	fprintf(fs, "%lf,", intp[i][j]);
		//}
		//sprintf(str, "%f, %f, %f, %f, %f, %f", intp[0][j], intp[1][j], intp[2][j], intp[3][j], intp[4][j], intp[5][j]);
		//RtPrintf("%s\n", str);
		//memset(str, 0, sizeof(char) * 256);

		//fprintf(fs, "\n");
	}
	free(smooth_sine);

	if (fs != nullptr)
		fclose(fs);
}

void icalab::SinusoidalTest::StartSinusoidalTest(void) {
	for (int i = 0; i < 6; i++)
		SetAxisControlMode(i, McControlMode::modeDirectPos);
	RtSetTimerRelative(hTimer_sine_test, &liPeriod_sineTest, &liPeriod_sineTest);
	this->rt_thread_started_ack = true;
}


void icalab::SinusoidalTest::setSinusoidalTestISR(double _clk_ms) {
	liPeriod_sineTest.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	hTimer_sine_test = RtCreateTimer(NULL, 0, RtSinusoidalTestRoutine, this, RT_PRIORITY_MAX - 2, CLOCK_FASTEST);
}
//
void icalab::SinusoidalTest::ConvertToAbsJointPos(double* pulse_init) {
	for (int i = 0; i < 6; i++) {
		for (unsigned int j = 0; j < intp_length; ++j) {
			intp[i][j] = pulse_init[i] + PPU_DEG * this->ratio[i] * intp[i][j];
		}
	}
}


//  
void RTFCNDCL RtSinusoidalTestRoutine(void* _context) {
	icalab::SinusoidalTest* self = static_cast<icalab::SinusoidalTest*>(_context);
	if (self->rt_thread_started_ack) {
		RtPrintf("Start Sinusoidal Test routine. (Thread ID: 0x%#04x)\n", GetCurrentThreadId());
		self->rt_thread_started_ack = false;
	}
	// Start CSP:
	for (int i = 0; i < 6; i++) {
		SetAxisPosition(i, self->intp[i][self->routine_index]);
	}

	//SetServoTargetPosition(self->frequency_sweep.joint_index, self->intp[self->routine_index]); // legacy
	++(self->routine_index);
	if (self->routine_index >= self->intp_length) {
		RtPrintf("Finishing Sinusoidal Test\n");
		self->routine_index = 0;
		self->rt_thread_running = false;
		//SetServoControlMode(self->routine_index, McConr::modeMasterIntPos);//legacy
		for (int i = 0; i < 6; i++)
			SetAxisControlMode(i, McControlMode::modeMasterIntPos);
		if (self->hTimer_sine_test != NULL) {
			RtCancelTimer(self->hTimer_sine_test, NULL);
		}
	}
}

#pragma endregion SinusoidalTest