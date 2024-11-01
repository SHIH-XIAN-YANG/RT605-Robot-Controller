#ifndef __CHIRP_H__
#define __CHIRP_H__
#include<algorithm>
#include<vector>
#define Frequency_Signal false
#define TimeDomain_Signal true
namespace ICAL {
	namespace signal {
		// Kind of frequency sweep
		enum class FreqDomain : unsigned short { 
			linear = 0,
			gaussian = 1
		};
		// - Time-domain Function/ Excitation Signal
		enum class TimeDomain : unsigned short { 
			cosine = 0, 
			sin = 1
		};
		//
		template<class T>
		class chirp {
		private:
			// frequency-swept funciton:
			inline T linear(T time) {
				return _f0 + _beta * time;
			}
			inline T gaussian(T time) {
				return _f0 + _beta  *static_cast<T>(exp(-(pow(time - _t1/2, 2) / (2.0 * _alpha * _alpha))));
			}
		protected:			
			unsigned int _len; 
			// --------------- Time-domain attributes: ---------------
			TimeDomain _excitation;
			T (*y) (T wt); // time-domain excitation function pointer ( y(wt) )
			T _ts;          // sampling time
			T _t1;          // time period
			T _Amp; // excitation amplitude
			T _bias;			
			// --------------- Frequency-domain attributes: ---------------
			FreqDomain _sweep;  // kind of frequency sweep
			T (ICAL::signal::chirp<T>::*f)(T time); // frequency-domain function pointer, i.e. frequency is the function of time ( Freq(t) )
			T _f0;          // starting frequency
			T _f1;          // ending frequency
			T _beta; // for linear function: slope; for gaussian funcion: length of frequency band 
			T _alpha; // if sweep == gaussian: standard deviation
			std::vector<T>* _signal_Buffer;
			
		public:
			bool _DecCondition;
			chirp(void) { };
			/*
			  Parameters:
				- excitation<enum ICAL::signal::TimeDomain>: excitaion signal in time-domain		  
				- ts: sampling time, 
				- t1: time period
				- amp: excitation amplitude
				- bias: signal bias

				- method<eunum ICAL::signal::FreqDomain>: kind of frequency sweep
				- f0: starting frequency
				- f1: ending frequency				
				- alpha: optional, default: 1 (gaussian: frequency band)
			*/
			void setup(TimeDomain excitation, T ts, T t1, T amp, T bias,
				       FreqDomain method, T f0, T f1, T alpha = 1) {
				_ts = ts; _t1 = t1; _f0 = f0; _f1 = f1; 
				_Amp = amp;
				_bias = bias;
				_sweep = method;
				_excitation = excitation;
				_len = static_cast<unsigned int>(_t1 / ts) + 1;
				// 
				switch (this->_sweep) {
					case ICAL::signal::FreqDomain::linear: {
						_beta = (f1 - f0) / _t1; // slope of the linear function
						this->f = &ICAL::signal::chirp<T>::linear;
						break;
					}
					case ICAL::signal::FreqDomain::gaussian: {
						_alpha = alpha; // standard deviation
						_beta = f1 - f0;
						this->f = &ICAL::signal::chirp<T>::gaussian;
						break;
					}
				}
				switch (this->_excitation) {
					case TimeDomain::cosine: {
						this->y = cos;
						break;
					}
					case TimeDomain::sin: {
						this->y = sin;
						break;
					}
				}
			}
			//
			/*
			*/
			bool generate_signal(bool signalDomain, std::vector<T>& sig, bool dec_condition) {
				if (_sweep == FreqDomain::linear && dec_condition == true)
					_DecCondition = true;
				else
					_DecCondition = false;
				if(_DecCondition == false){
					sig.reserve(_len); sig.resize(_len);
					for (unsigned int i = 0; i < _len; ++i) {
						T&& t = i * _ts;
						if (signalDomain == TimeDomain_Signal)
							sig.at(i) = _bias + _Amp * (*y)(6.28318530718f * t * (this->*f)(t));
						else
							sig.at(i) = (this->*f)(t);
					}
				}
				else if (_DecCondition == true) {
					unsigned size = 2 * _len;
					sig.reserve(size); sig.resize(size);
					for (unsigned int i = 0; i < _len; ++i) {
						T&& t = i * _ts;
						if (signalDomain == TimeDomain_Signal)
							sig.at(i) =  _bias + _Amp * (*y)(6.28318530718f * t * (this->*f)(t));
						else
							sig.at(i) = (this->*f)(t);
					}
					T t1_end = _ts * _len;
					// Reverse:
					T f0_tmp = _f0;
					_f0 = _f1;
					_beta *= -1;
					for (unsigned int i = 0; i < _len; ++i) {
						T&& t = i * _ts;
						T&& tau = t - t1_end;
						if (signalDomain == TimeDomain_Signal)
							sig.at(i+_len) = _bias + _Amp * (*y)(6.28318530718f * t * (this->*f)(tau));
						else
							sig.at(i + _len) = (this->*f)(t);
					}
					_f0 = f0_tmp;
					_beta *= -1;
				}
				return true;
			}
		// end class
			/* Example 1: (Generate chirp-sine signal, where applied linear function to sweep the frequency)
				float ts{ 0.001 }, t1{ 5 }, f0{ 0 }, f1{ 20 };
				ICAL::signal::chirp<float> Freq_sweep1;
				Freq_sweep1.setup(ICAL::signal::TimeDomain::cosine, ts, t1, 5.0f, 0.0f,
					ICAL::signal::FreqDomain::gaussian, f0, f1);
				std::vector<float> yy;
				Freq_sweep1.generate_signal(yy);
				fs.open("d:/yy.csv", std::fstream::out);
				for (unsigned int i = 0; i < yy.size(); ++i) {
					fs << yy.at(i);
					if (i != yy.size() - 1)
						fs << "\n";
				}
				fs.close();
			*/
		};
	}
}
#endif
