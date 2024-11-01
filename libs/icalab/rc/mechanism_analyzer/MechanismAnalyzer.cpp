#include "MechanismAnalyzer.h"


//
icalab::MechanismAnalyzer::~MechanismAnalyzer(void) {

}
//
icalab::MechanismAnalyzer::MechanismAnalyzer(void) {
	Initial();
}
//
void icalab::MechanismAnalyzer::Initial(void) {
	this->intp = nullptr;
	this->rt_thread_started_ack = false;
	this->routine_index = 0;
	this->rt_thread_running = false;
}

//
void icalab::MechanismAnalyzer::GenerateSignal(void) {
	// generate signal:
	double f = 0.0;
	double time = 0.0;
	
	FILE* fs = nullptr;
	fopen_s(&fs, "e:/sweep.csv", "w+");
	fprintf(fs, "time,f,y\n");
	frequency_sweep.time_signal_type = SignalType::sin;
	frequency_sweep.freq_sweep_type = SignalType::linear;
	switch (frequency_sweep.time_signal_type) {
		case SignalType::sin: {
			if (frequency_sweep.freq_sweep_type == SignalType::linear) {
				RtPrintf("sin-linear\n");
				for (unsigned int i = 0; i < this->intp_length; ++i) {
					time = (double)i * frequency_sweep.ts*0.001; // unit: second
					f = this->Linear(time, frequency_sweep.sweep_slope, frequency_sweep.f0);
					this->intp[i] = frequency_sweep.arg * sin(2.0 * M_PI * f * time);
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
	//GetServoPosition(frequency_sweep.joint_index, &pulse_init); // legacy
	KsError nRet;
	nRet = GetAxisPosition(frequency_sweep.joint_index, McSource::mcActualValue, &pulse_init); // Not sure if it is right
	if (nRet != errNoError) {
		RtPrintf("Get Axix Position Error!!: %d\n", nRet);
	}
	this->ConvertToAbsJointPos(pulse_init);
} 
void icalab::MechanismAnalyzer::Setup(FreqSweep prt) {
	memcpy(&this->frequency_sweep, &prt, sizeof(FreqSweep));
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
	this->frequency_sweep.sweep_slope = (frequency_sweep.f1 - frequency_sweep.f0) / (frequency_sweep.tf*0.001);
	RtPrintf("f slope: %d mHz/sec.\n", (int)(frequency_sweep.sweep_slope * 1000.0));
	// Setup ISR:
	this->setFrequencySweepISR(prt.ts);
}
//
void icalab::MechanismAnalyzer::setFrequencySweepISR(double _clk_ms) {
	liPeriod_freq_sweep.QuadPart = static_cast<LONGLONG>(_clk_ms * 10000.0);
	hTimer_freq_sweep = RtCreateTimer(NULL, 0, RtFrequencySweepRoutine, this, RT_PRIORITY_MAX - 2, CLOCK_FASTEST);
}
//
void icalab::MechanismAnalyzer::ConvertToAbsJointPos(double pulse_init) {
	double ratio[6] = { 80.0, 100.0, -80.0, -81.0, -80.0, -50.0 };
	for (unsigned int i = 0; i < intp_length; ++i) {
		intp[i] = pulse_init + PPU_DEG * ratio[frequency_sweep.joint_index] * intp[i];
	}
}
//
double icalab::MechanismAnalyzer::Linear(double t, double slope, double bias) {
	return slope * t + bias;
}
double icalab::MechanismAnalyzer::Gaussian(double t, double std, double var) {
	return exp(t);
}
//
void icalab::MechanismAnalyzer::StartFrequencySweep(void) {
	//SetServoControlMode(frequency_sweep.joint_index, ControlMode::modeDirectPos); // legacy
	SetAxisControlMode(frequency_sweep.joint_index, McControlMode::modeDirectPos);
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
	WriteAxisTargetPosition(self->frequency_sweep.joint_index, self->intp[self->routine_index]);
	//SetServoTargetPosition(self->frequency_sweep.joint_index, self->intp[self->routine_index]); // legacy
	++(self->routine_index);
	if (self->routine_index >= self->intp_length) {
		RtPrintf("Finishing freq. sweep.\n");
		self->routine_index = 0;
		self->rt_thread_running = false;
		//SetServoControlMode(self->routine_index, McConr::modeMasterIntPos);//legacy
		SetAxisControlMode(self->frequency_sweep.joint_index, McControlMode::modeMasterIntPos);
		RtCancelTimer(self->hTimer_freq_sweep, NULL);
	}
}
