#ifndef __BRT_WIN32_TIME_h__
#define __BRT_WIN32_TIME_h__

#include<Windows.h>
#define TICK_COUNTER_PER_MILLI_SECOND 1000
namespace brt{
	class PerformanceCounter{
		private:
			LARGE_INTEGER m_liPerfFreq;
			LARGE_INTEGER m_liPerfInit;
			LARGE_INTEGER m_liPerfTick;
			
			double m_cpu_freq;
		public:
			PerformanceCounter(void){
				QueryPerformanceFrequency(&m_liPerfFreq);
				m_cpu_freq = static_cast<double>(m_liPerfFreq.QuadPart)/1000.0;
			}
			inline void TimerSet(void){
				QueryPerformanceCounter(&m_liPerfInit);	
			}
			inline double TimerWatch_ms(void){
				QueryPerformanceCounter(&m_liPerfTick);
				return static_cast<double>((-m_liPerfInit.QuadPart + m_liPerfTick.QuadPart)/m_cpu_freq);
			}
			
	};
};
#endif