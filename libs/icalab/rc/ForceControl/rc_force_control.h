/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2023/05/28, b.r.tseng
*/
#ifndef __RC_FORCE_CONTROL_H__
#define __RC_FORCE_CONTROL_H__

#include<windows.h>
#include<stdio.h>
#include<string.h>
#ifdef UNDER_RTSS
//#include"Rtapi.h"
//#include"ksm64shared.h"
#include <rtapi.h>
#elif defined WINDOWS_DEBUGGING
#include"stdio.h"
#endif

#include<array>

#include"rc_force_control_macro.h"
#include"../../dsp/filter.h"
#include"rc_force_control_macro.h"
//#include"../../../../Rt_ICRC/robot_controller_macro.h"


namespace icalab{
	//
	enum class ForceController :unsigned short {
		Impedance1,
		PID,
		Admittance
	};
	//
	enum class FcMode :unsigned short {
		Ready,
		ToStartPoint,
		FindSurface,
		Acc,
		Tracking,
		Dec,
		LeaveSurface,
		End,
		Stop // for error situation
	};
	//
	template<class T> 
	class FcImpedance1{
		private:
			bool tmp;
		protected:
			// 系統資訊：
			FcMode fc_mode;
			unsigned long long fc_step;
			unsigned long long fc_start_point;
			unsigned long long fc_end_point;
			unsigned long long fc_process_section[2];
			filter::MovingAverage output_filter;
			T ef ;
			T ef_k_1 ;
			T def; // error force 的梯度
			T ef_accumlated; // error force 的累積
			T xe;
			// 力量控制：
			T fc;
			T fd;
			T prePressed_force;
			// 控制參數：
			T km;
			T kff;
			T delta_xe_est;
			T delta_xe_lr;
			T output_lim;
			// 環境剛性估測：
			bool ke_estimator_en; // enable/disable
			bool xe_estimator_en;

			T ke_hat_init;
			T ke_hat;
			T ke_lr; // 學習率

			T invKm_plus_invKe;
			T invKmKe;
			// log:
			T log_delta_xf_tcp;
		public:
			FcImpedance1(void);
			~FcImpedance1(void);
			T getForceCommandValue(void);
			T getOutputValue(void);
			T getKeValue(void);
			FcMode getFcMode(void);
			void setControlProcessSectoin(unsigned long long start, unsigned long long end, unsigned long long process_start, unsigned long long process_end);
			void ResetState(void);
			void setFilter(unsigned short _order);
			T ForceCommand(unsigned long long& _fc_step);
			void StiffnessEstimator(T ef, T x, T xe);
			void setImpedanceGains(T km, T ke);
			void setControlTarget(T _fd, T _prePress_force, T _dx_lim);
			void EnableEstimator(bool _ke_est_en, bool _xe_est_en);
			T operator()(unsigned long long xr_index, T _fext, T x);

	};
	//

	template<class T> 
	class FcPID{
	private:
		bool tmp;
	protected:
		// 系統資訊：
		FcMode fc_mode;
		unsigned long long fc_step;
		unsigned long long fc_start_point;
		unsigned long long fc_end_point;
		unsigned long long fc_process_section[2];
		filter::MovingAverage output_filter;
		T ef;
		T def; // error force 的梯度
		T ef_accumlated; // error force 的累積
		T xe;
		// 力量控制：
		T fc;
		T fd;
		T prePressed_force;
		// 控制參數：
		T ef_k_1; // ef[k-1]

		T m_kp;
		T m_ki;
		T m_kd;

		bool m_anti_windup_en;
		T m_anti_windup_threshold_lower;
		T m_anti_windup_threshold_upper;

		// log:
		T log_delta_xf_tcp;
	public:	
		FcPID(void);
		~FcPID(void);
		T ForceCommand(unsigned long long& _fc_step);
		FcMode getFcMode(void);
		T getForceCommandValue(void);
		T getOutputValue(void);
		void getPidGain(T* pid);
		void ResetState(void);
		void setPID(T _kp, T _ki, T kd);
		void setPID(T* _k);
		void setControlTarget(T _fd, T _prePress_force, T _dx_lim);
		void setIntegralAntiWindup(bool enable_anti_windup, T threshold_L, T threshold_U);
		void AntiWindup(void);
		T getAccumulation(void); // 回傳 delta_Xf_tcp[k]
			
		T operator()(unsigned long long xr_index, T _fext);
	};
	//


/*  (待設計之自適應性參數調整控制器)
	template<class T>
	class FcFuzzyPID : public FcPID<T> {
	public:



	};
	//
	template<class T>
	class FcFuzzyImpedance : public FcImpedance1<T> {
	public:



	};
*/

}



#endif