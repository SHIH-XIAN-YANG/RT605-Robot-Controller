#include "MechanismAnalyzer.h"


icalab::MechanismAnalyzer::MechanismAnalyzer(void) {
	this->intp = nullptr;
	this->routine_index = 0;
	this->rt_thread_started_ack = false;
	this->rt_thread_running = false;
	
}

icalab::MechanismAnalyzer::~MechanismAnalyzer(void) {
	free(intp);
}

int icalab::MechanismAnalyzer::set_app_dir(char* app_dir) {
	return strcpy_s(this->app_dir, app_dir);
}

void icalab::MechanismAnalyzer::Setup(FreqSweep prt) {
	memcpy(&this->frequency_sweep_params, &prt, sizeof(FreqSweep));
	this->intp_length = static_cast<unsigned int>(prt.tf / prt.ts);
	RtPrintf("length: %d\n", intp_length);
	// initialize the INTP buffer:
	if (this->intp == nullptr) {
		this->intp = static_cast<double*>(malloc(sizeof(double) * intp_length));
	}
	else {
		free(this->intp);
		this->intp = static_cast<double*>(malloc(sizeof(double) * intp_length));
	}
	this->frequency_sweep_params.sweep_slope = (frequency_sweep_params.f1 - frequency_sweep_params.f0) / (frequency_sweep_params.tf * 0.001);
	RtPrintf("f slope: %d mHz/sec.\n", (int)(frequency_sweep_params.sweep_slope * 1000.0));
	// Setup ISR:
	this->setFrequencySweepISR(prt.ts);
}

void icalab::MechanismAnalyzer::generate_INTP_Signal(void){
	/* Generate chirp-Sine */
	double f = 0.0;
	double time = 0.0;

	errno_t err;
	FILE* fs = nullptr;
	char path[MAX_PATH] = { 0 };
	strcpy(path, app_dir);
	strcat(path, "data_log\\sweep_ref.csv");

	err = fopen_s(&fs, path, "w+");
	if (err != 0) {
		RtPrintf("Error: Could not open file: sweep.csv!!!\n");
		return;
	}
	fprintf(fs, "time,f,y\n");
	frequency_sweep_params.time_signal_type = SignalType::sin;
	frequency_sweep_params.freq_sweep_type = SignalType::linear;
	
	switch (frequency_sweep_params.time_signal_type) {
		case SignalType::sin: {
			if (frequency_sweep_params.freq_sweep_type == SignalType::linear) {
				RtPrintf("sin-linear\n");
				for (unsigned int i = 0; i < this->intp_length; ++i) {
					time = (double)i * frequency_sweep_params.ts * 0.001; // unit: second
					f = this->Linear(time, frequency_sweep_params.sweep_slope, frequency_sweep_params.f0);
					this->intp[i] = frequency_sweep_params.arg * sin(2.0 * M_PI * f * time);
					// export:
					if (fs != nullptr) {
						fprintf(fs, "%lf,%lf,%lf\n", time, f, intp[i]);
					}
				}
			}
			if (fs != nullptr)
				fclose(fs);
			break;
		}
	}
	// Convert to Abs. Joint Position:
	double pulse_init;
	KsError nRet;
	nRet = GetAxisPosition(frequency_sweep_params.joint_index, McSource::mcActualValue, &pulse_init); // Not sure if it is right
	if (nRet != errNoError) {
		RtPrintf("Get Axix Position Error!!: %d\n", nRet);
	}
	this->ConvertToAbsJointPos(pulse_init);

}

void icalab::MechanismAnalyzer::setFrequencySweepISR(double _clk_ms){
	liPeriod_freq_sweep.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	hTimer_freq_sweep = RtCreateTimer(NULL, 0, RtFrequencySweepRoutine, this, RT_PRIORITY_MAX - 2, CLOCK_FASTEST);
}

double icalab::MechanismAnalyzer::Linear(double t, double slope, double bias){ return slope * t + bias; }

double icalab::MechanismAnalyzer::Gaussian(double t, double std, double var){ return exp(t); }

void icalab::MechanismAnalyzer::ConvertToAbsJointPos(double pulse_init) {
	double ratio[6] = { 80.0, 100.0, -80.0, -81.0, -80.0, -50.0 }; //Reduction Ratio
	for (unsigned int i = 0; i < intp_length; ++i) {
		intp[i] = pulse_init + PPU_DEG * ratio[frequency_sweep_params.joint_index] * intp[i];
	}
}

void icalab::MechanismAnalyzer::StartFrequencySweep(void) {
	SetAxisControlMode(frequency_sweep_params.joint_index, McControlMode::modeDirectPos); // CiA402 Cyclic Synchronous Position (CSP)
	//SetServoControlMode(frequency_sweep.joint_index, ControlMode::modeDirectPos);
	RtSetTimerRelative(hTimer_freq_sweep, &liPeriod_freq_sweep, &liPeriod_freq_sweep);
	this->rt_thread_started_ack = true;
}

//  
void RTFCNDCL RtFrequencySweepRoutine(void* _context) {
	icalab::MechanismAnalyzer* self = static_cast<icalab::MechanismAnalyzer*>(_context);
	if (self->rt_thread_started_ack) {
		RtPrintf("Start Frequency-sweep routine. (Thread ID: 0x%#04x)\n", GetCurrentThreadId());
		self->rt_thread_started_ack = false;
	}
	// Start CSP:
	SetAxisPosition(self->frequency_sweep_params.joint_index, self->intp[self->routine_index]);
	//SetServoTargetPosition(self->frequency_sweep.joint_index, self->intp[self->routine_index]); // legacy
	++(self->routine_index);
	if (self->routine_index >= self->intp_length) {
		RtPrintf("Finishing freq. sweep.\n");
		self->routine_index = 0;
		self->rt_thread_running = false;
		//SetServoControlMode(self->routine_index, McConr::modeMasterIntPos);//legacy
		SetAxisControlMode(self->frequency_sweep_params.joint_index, McControlMode::modeMasterIntPos);
		if (self->hTimer_freq_sweep != NULL) {
			RtCancelTimer(self->hTimer_freq_sweep, NULL);
		}
	}
}