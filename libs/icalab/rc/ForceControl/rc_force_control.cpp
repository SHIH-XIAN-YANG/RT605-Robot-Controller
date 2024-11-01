/*
	- Create: 2022/08/25, b.r.tseng
	- Edit: 2023/05/28, b.r.tseng
*/
#include"rc_force_control.h"

// (舊版本，暫時移除)
#pragma region ForceControllerBase
/* 
template<class T> icalab::ForceController<T>::ForceController(void){
	m_robot_q = nullptr;
	m_force_sensor = nullptr;
	m_robot_x_tcp = nullptr;
	m_robot_xd = nullptr;
}
//
template<class T> icalab::ForceController<T>::~ForceController(void){
}
// 設定力量控制演算法：
template<class T> void icalab::ForceController<T>::set_FcAlgorithm(FcAlgorithm _fcAlgorithm){
	
}
// 查看目前所使用的力量控制演算法：
template<class T> FcAlgorithm icalab::ForceController<T>::check_FcAlgorithm(void){
	return FcAlgorithm::Impedance1;
}
//
template<class T> unsigned short icalab::ForceController<T>::set_setting(unsigned short _index, unsigned short _set){
	
	unsigned short* pSetting_register = (m_setting + _index);

	*pSetting_register = _set;
	
	return *pSetting_register;
}
//
template<class T> unsigned short icalab::ForceController<T>::check_setting(unsigned short _index){
	unsigned short ret{0};
	if(_index < 4){
		ret = m_setting[_index];
	}
	return ret;
}
//
template<class T> void icalab::ForceController<T>::GravityCompensation(bool _open){
	if( (m_setting[0] & FC_G_COMPENSATE) == FC_G_COMPENSATE){ // has already been turned on
		if(_open){ // 在 "已經開啟" 重力補償的情況下，下達 "開起" 的命令：
			RtPrintf("Turn on the FC_GravityCompensation.\n");
		}
		else{     // 在  "已經開啟" 重力補償的情況下，下達 "關閉" 的命令：
			this->set_setting(0, m_setting[0] ^ FC_G_COMPENSATE);
			RtPrintf("Turn off the FC_GravityCompensation.\n");
		}		
	}
	else{ // does not been turned on yet
		if(_open){ // 在 "未開啟" 重力補償的情況下，下達 "開起" 的命令：
			this->set_setting(0, m_setting[0] ^ FC_G_COMPENSATE);
			RtPrintf("Turn on the FC_GravityCompensation.\n");
		}
		else{     // 在  "未開啟" 重力補償的情況下，下達 "關閉" 的命令：	
			RtPrintf("Turn off the FC_GravityCompensation.\n");
		}	
	}
}
//
template<class T> bool icalab::ForceController<T>::FcON(bool _open){
	bool ret = false;
	if( (m_setting[0] & FC_ON) == FC_ON){ // has already been turned on
		if(_open){ // 在 "已經開啟" 的情況下，下達 "開起" 的命令：
			RtPrintf("Turn on the force control.\n");
		}
		else{     // 在  "已經開啟" 的情況下，下達 "關閉" 的命令：
			this->set_setting(0, m_setting[0] ^ FC_ON);
			RtPrintf("Turn off the FC_GravityCompensation.\n");
		}		
	}
	else{ // does not been turned on yet
		if(_open){ // 在 "未開啟" 的情況下，下達 "開起" 的命令：
			this->set_setting(0, m_setting[0] ^ FC_ON);
			RtPrintf("Turn on the force control.\n");
		}
		else{     // 在  "未開啟" 的情況下，下達 "關閉" 的命令：	
			RtPrintf("Turn off the force control.\n");
		}	
	}
	return ret;
}
//
template<class T> void icalab::ForceController<T>::ComputeTransformMatrix(T* _set_transform_vec, T* _ret_transform_mat){
	
}
//
template<class T> void icalab::ForceController<T>::CoordinateCalibration(bool _calType, T* _transform_vec){
	
}
// 設定力量控制方塊 輸出的限制：
template<class T> void icalab::ForceController<T>::set_FcOutputLimt(unsigned short _op_lim_unit, T* _output_lim){
	unsigned short setting_tmp = m_setting[1] & (~FC_SET1_OP_LIM);
	setting_tmp |= _op_lim_unit;
	this->set_setting(1, setting_tmp);
	
	if((m_setting[1] & FC_SET1_OP_LIM) == FC_LIM_CLOSE){
		for(unsigned short i = 0; i<6; ++i)
			m_outPut_lim[i] = _output_lim[i];
	}
}
*/
#pragma endregion ForceControllerBase
//
#pragma region ForceControlleImpedance1
//
template<class T> icalab::FcImpedance1<T>::FcImpedance1(void) {
	ResetState();
	this->output_filter.setup(30); // 設定 29 階的 Moving average filter
}
//
template<class T> icalab::FcImpedance1<T>::~FcImpedance1(void) {
	ResetState();
}
//
template<class T> T icalab::FcImpedance1<T>::getForceCommandValue(void) {
	return fc;
}
//
template<class T> T icalab::FcImpedance1<T>::getOutputValue(void) {
	return log_delta_xf_tcp;
}
//
template<class T> icalab::FcMode icalab::FcImpedance1<T>::getFcMode(void) {
	return this->fc_mode;
}
//
template<class T> void icalab::FcImpedance1<T>::ResetState(void) {
	this->xe = 0.0;
	this->fc_step = 0;
	this->fc = 0.0;
	this->ef = 0.0;
	this->ef_k_1 = 0.0;
	this->def = 0.0; 
	this->ef_accumlated = 0.0; 
	this->fc_mode = icalab::FcMode::Ready;
	this->tmp = false;
}
//
template<class T> void icalab::FcImpedance1<T>::setControlProcessSectoin(unsigned long long start, unsigned long long end, unsigned long long process_start, unsigned long long process_end) {
	this->fc_start_point = start;
	this->fc_end_point = end;
	this->fc_process_section[0] = process_start;
	this->fc_process_section[1] = process_end;
}
//
template<class T> T icalab::FcImpedance1<T>::ForceCommand(unsigned long long& _fc_step) {
	if (fc_mode == icalab::FcMode::Acc || fc_mode == icalab::FcMode::Tracking) {
		return prePressed_force + (fd - prePressed_force) * (1.0 - exp(-((double)_fc_step) / 600.0)); // Acc. time = 3.0 seconds
	}
	else if (fc_mode == icalab::FcMode::Dec) {
		return fd * (exp(-(double)_fc_step / 600.0));
	}
}
//
template<class T> void icalab::FcImpedance1<T>::StiffnessEstimator(T ef, T x, T xe) {

}
//
template<class T> void icalab::FcImpedance1<T>::setControlTarget(T _fd, T _prePress_force, T _dx_lim) {
	this->fd = _fd;
	this->prePressed_force = _prePress_force;

	// output limit 單位從 GUI 的 mm/sec. 轉換成 m/ms
	// => mm/sec. = (10^-3 / 10^3) * (m/ms)
	// ex: b [mm/sec.] = b*10^-6 [m/ms], where b is a real number
	this->output_lim = _dx_lim/1000000.0;
}
//
template<class T> void icalab::FcImpedance1<T>::setImpedanceGains(T _km, T _ke) {
	this->km = _km;
	this->ke_hat = _ke;

	this->invKm_plus_invKe = (T)1.0 / km + (T)1.0 / ke_hat;
	this->invKmKe = ke_hat / km;
}
//
template<class T> void icalab::FcImpedance1<T>::EnableEstimator(bool _ke_est_en, bool _xe_est_en) {
	this->ke_estimator_en = _ke_est_en;
	this->xe_estimator_en = _xe_est_en;
}
//
template<class T> T icalab::FcImpedance1<T>::operator()(unsigned long long xr_index, T _fext, T x) {
	T delta_xf_tcp = 0.0;
	fc = ForceCommand(fc_step);
	// 儲存前一步的 ef，等下可以做微分計算
	ef_k_1 = ef;
	// 計算 error force:
	ef = fc - _fext;
	if (abs(ef) <= 0.1) { // 力量控制的精度暫且設定在 0.1 N
		if (ef > 0.0)
			ef = 0.00001;
		else if (ef <= 0.0)
			ef = -0.00001;
	}
	this->def = (ef - ef_k_1)/ (MOTION_CYCLE / 1000.0);
	// 更新力量控制的階段：
	switch (this->fc_mode) {
		case icalab::FcMode::Ready: {
			RtPrintf("Ready mode\n");
			fc_mode = icalab::FcMode::ToStartPoint;
			// 更新力量控制輸出：
			delta_xf_tcp = 0.0;
			break;
		}
		case icalab::FcMode::ToStartPoint: {
			if (tmp == false) {
				printf("\n ToStartPoint mode\n");
				tmp = true;
			}
			if (xr_index == fc_start_point) {
				RtPrintf("FindSurface mode\n");
				fc_mode = icalab::FcMode::FindSurface;
				tmp = false;
			}
			// 更新力量控制輸出：
			delta_xf_tcp = 0.0;
			break;
		}
		case icalab::FcMode::FindSurface: {
			if (tmp == false) {
				printf("\n Making contact force for %i mN .....\n", (int)(prePressed_force * 1000.0));
				tmp = true;
			}
			if (_fext <= prePressed_force) {
				this->prePressed_force = _fext; // 修改預壓力為當前的受力，因為接觸當下有可能會超過預設的預壓力。
				fc_mode = icalab::FcMode::Tracking;
				printf("\n Start Force Control ... \n ");
				this->xe = x;
				tmp = false;
			}
			// 更新力量控制輸出：
			delta_xf_tcp = 0.000005;
			break;
		}
		case icalab::FcMode::Tracking: {
			if (tmp == false) {
				printf("\n Force tracking for %i mN .....\n", (int)(fd * 1000.0));
				tmp = true;
			}
			// 更新力量控制輸出：
			//delta_xf_tcp = -(invKm_plus_invKe * ef +invKmKe * (x - xe)); // 這部分還有些問題，先暫時簡化
			delta_xf_tcp =  -ef*(1/km + 1/ke_hat);
			// 限制輸出量：
			if (abs(delta_xf_tcp) >= output_lim) {
				if (delta_xf_tcp > 0)
					delta_xf_tcp = output_lim;
				else
					delta_xf_tcp = -output_lim;
			}
			// 力量控制區域 Acc/Tracking -> Dec 的階段切換：
			if (xr_index == fc_process_section[1]) { // 位置控制迴路的 xr 已經更新到最後一個力量控制的點位
				fc_mode = icalab::FcMode::Dec;
				fc_step = 0;
				tmp = false;
			}
			else { // 位置控制迴路的 xr 還沒更新到最後一個力量控制的點位，所以還在 Acc 或 Tracking 階段
				// 更新力量控制index:
				++fc_step;
			}
			//
			break;
		}
		case icalab::FcMode::Dec:{
			if (!tmp) {
				printf("Force control mode: Dec.\n");
				tmp = true;
			}
			// 更新力量控制輸出：
			delta_xf_tcp = -ef * (1 / km + 1 / ke_hat);
			// 限制輸出量：
			if (abs(delta_xf_tcp) >= output_lim) {
				if (delta_xf_tcp > 0)
					delta_xf_tcp = output_lim;
				else
					delta_xf_tcp = -output_lim;
			}

			if (_fext >= prePressed_force) {
				fc_mode = icalab::FcMode::LeaveSurface;
				fc_step = 0;
				delta_xf_tcp = 0.0;
				tmp = false;
			}
			else {
				++fc_step;
			}
			break;
		}
		case icalab::FcMode::LeaveSurface: {
			if (tmp == false) {
				printf("\n Leaving surface ......\n");
				tmp = true;
			}
			if (fc_step >= 1000) {
				printf("\n Complete force-control process.\n");
				fc_mode = icalab::FcMode::End;
				tmp = false;
			}
			else {
				// 更新力量控制輸出：
				delta_xf_tcp = -0.00002;
				++fc_step;
			}
			break;
		}
		case icalab::FcMode::End: {
			if (tmp == false) {
				printf("\n Complete force-control process.\n");
				tmp = true;
			}
			// 更新力量控制輸出：
			delta_xf_tcp = 0.0;
			break;
		}
		default: {
			// do nothing
			// 更新力量控制輸出：
			delta_xf_tcp = 0.0;
			break;
		}
	}
	//
	log_delta_xf_tcp = output_filter(delta_xf_tcp);
	return log_delta_xf_tcp;
}
//
template<class T> T icalab::FcImpedance1<T>::getKeValue(void) {
	return ke_hat;
}
//
template<class T> void icalab::FcImpedance1<T>::setFilter(unsigned short _order) {
	this->output_filter.setup(_order);
}
#pragma endregion ForceControllerImpedance1
//
#pragma region PID_Controller
template<class T> icalab::FcPID<T>::FcPID(void) {
	ResetState();
	m_kp = static_cast<T>(0.0);
	m_ki = static_cast<T>(0.0);
	m_kd = static_cast<T>(0.0);
	output_filter.setup(30);
}
//
template<class T> icalab::FcPID<T>::~FcPID(void) {

}
//
template<class T> icalab::FcMode icalab::FcPID<T>::getFcMode(void) {
	return this->fc_mode;
}
//
template<class T> T icalab::FcPID<T>::getForceCommandValue(void) {
	return fc;
}
//
template<class T> T icalab::FcPID<T>::getOutputValue(void) {
	return log_delta_xf_tcp;
}
//
template<class T> T icalab::FcPID<T>::ForceCommand(unsigned long long& _fc_step) {
	if (fc_mode == icalab::FcMode::Acc || fc_mode == icalab::FcMode::Tracking) {
		return prePressed_force + (fd - prePressed_force) * (1.0 - exp(-((double)_fc_step) / 600.0)); // Acc. time = 3.0 seconds
	}
	else if (fc_mode == icalab::FcMode::Dec) {
		return fd * (exp(-(double)_fc_step / 600.0));
	}
}
//
template<class T> void icalab::FcPID<T>::ResetState(void) {
	this->ef = 0.0;
	this->ef_k_1 = static_cast<T>(0.0);
	this->ef_accumlated = static_cast<T>(0.0);
	this->tmp = false;
}
//
template<class T> void icalab::FcPID<T>::setPID(T _kp, T _ki, T _kd) {
	m_kp = _kp;
	m_ki = _ki;
	m_kd = _kd;
}
//
template<class T> void icalab::FcPID<T>::setPID(T* _k) {
	m_kp = _k[0];
	m_ki = _k[1];
	m_kd = _k[2];
}
//
template<class T> void icalab::FcPID<T>::setIntegralAntiWindup(bool enable_anti_windup, T threshold_L, T threshold_U) {
	m_anti_windup_en = true;
	m_anti_windup_threshold_lower = threshold_L;
	m_anti_windup_threshold_upper = threshold_U;
}
//
template<class T> void icalab::FcPID<T>::AntiWindup(void) {
	if (m_anti_windup_en) {
		if (abs(ef) >= m_anti_windup_threshold_upper) {
			if (ef >= 0.0)
				ef_accumlated = m_anti_windup_threshold_upper;
			else
				ef_accumlated = -m_anti_windup_threshold_upper;
		}
		else {
			ef_accumlated = 0.0;
		}
	}
}
//
template<class T> T icalab::FcPID<T>::getAccumulation(void) {
	return ef_accumlated;
}
//
template<class T> T icalab::FcPID<T>::operator()(unsigned long long xr_index, T _fext) {
	T delta_xf_tcp = 0.0;
	fc = ForceCommand(fc_step);
	// 儲存前一步的 ef，等下可以做微分計算
	ef_k_1 = ef;
	// 計算 error force:
	ef = fc - _fext;
	if (abs(ef) <= 0.1) { // 力量控制的精度暫且設定在 0.1 N
		if (ef > 0.0)
			ef = 0.00001;
		else if (ef <= 0.0)
			ef = -0.00001;
	}
	this->def = (ef - ef_k_1) / (MOTION_CYCLE / 1000.0);
	// 更新力量控制的階段：
	switch (this->fc_mode) {
	case icalab::FcMode::Ready: {
		fc_mode = icalab::FcMode::ToStartPoint;
		// 更新力量控制輸出：
		delta_xf_tcp = 0.0;
		break;
	}
	case icalab::FcMode::ToStartPoint: {
		if (tmp == false) {
			printf("\n ToStartPoint mode\n");
			tmp = true;
		}
		if (xr_index == fc_start_point) {
			RtPrintf("FindSurface mode\n");
			fc_mode = icalab::FcMode::FindSurface;
			tmp = false;
		}
		// 更新力量控制輸出：
		delta_xf_tcp = 0.0;
		break;
	}
	case icalab::FcMode::FindSurface: {
		if (tmp == false) {
			printf("\n Making contact force for %i mN .....\n", (int)(prePressed_force * 1000.0));
			tmp = true;
		}
		if (_fext <= prePressed_force) {
			this->prePressed_force = _fext; // 修改預壓力為當前的受力，因為接觸當下有可能會超過預設的預壓力。
			fc_mode = icalab::FcMode::Tracking;
			printf("\n Start Force Control ... \n ");
			tmp = false;
		}
		// 更新力量控制輸出：
		delta_xf_tcp = 0.000005;
		break;
	}
	case icalab::FcMode::Tracking: {
		if (tmp == false) {
			printf("\n Force tracking for %i mN .....\n", (int)(fd * 1000.0));
			tmp = true;
		}
		// 反積分終結：
		AntiWindup();
		// 更新力量控制輸出：
		ef_accumlated = (0.5 * (ef + ef_k_1) * 0.001); // trapzoidal integration
		delta_xf_tcp = m_kp * ef + m_kd * ((ef - ef_k_1) / 0.001) + m_ki * ef_accumlated;
		ef_k_1 = ef;

		// 力量控制區域 Acc/Tracking -> Dec 的階段切換：
		if (xr_index == fc_process_section[1]) { // 位置控制迴路的 xr 已經更新到最後一個力量控制的點位
			fc_mode = icalab::FcMode::Dec;
			fc_step = 0;
			tmp = false;
		}
		else { // 位置控制迴路的 xr 還沒更新到最後一個力量控制的點位，所以還在 Acc 或 Tracking 階段
			// 更新力量控制index:
			++fc_step;
		}
		//
		break;
	}
	case icalab::FcMode::Dec: {
		if (!tmp) {
			printf("Force control mode: Dec.\n");
			tmp = true;
		}
		// 反積分終結：
		AntiWindup();
		// 更新力量控制輸出：
		ef_accumlated = (0.5 * (ef + ef_k_1) * 0.001); // trapzoidal integration
		delta_xf_tcp = m_kp * ef + m_kd * ((ef - ef_k_1) / 0.001) + m_ki * ef_accumlated;
		ef_k_1 = ef;

		//
		if (_fext >= prePressed_force) {
			fc_mode = icalab::FcMode::LeaveSurface;
			fc_step = 0;
			delta_xf_tcp = 0.0;
			tmp = false;
		}
		else {
			++fc_step;
		}
		break;
	}
	case icalab::FcMode::LeaveSurface: {
		if (tmp == false) {
			printf("\n Leaving surface ......\n");
			tmp = true;
		}
		if (fc_step >= 1000) {
			printf("\n Complete force-control process.\n");
			fc_mode = icalab::FcMode::End;
			tmp = false;
		}
		else {
			// 更新力量控制輸出：
			delta_xf_tcp = -0.00002;
			++fc_step;
		}
		break;
	}
	case icalab::FcMode::End: {
		if (tmp == false) {
			printf("\n Complete force-control process.\n");
			tmp = true;
		}
		// 更新力量控制輸出：
		delta_xf_tcp = 0.0;
		break;
	}
	default: {
		// do nothing
		// 更新力量控制輸出：
		delta_xf_tcp = 0.0;
		break;
	}
	}
	//
	log_delta_xf_tcp = output_filter(delta_xf_tcp);
	return log_delta_xf_tcp;
}
//
template<class T> void icalab::FcPID<T>::setControlTarget(T _fd, T _prePress_force, T _dx_lim) {
	this->fd = _fd;
	this->prePressed_force = _prePress_force;
	//this->output_lim = _dx_lim;
}
//
template<class T> void icalab::FcPID<T>::getPidGain(T* pid) {
	pid[0] = m_kp;
	pid[1] = m_ki;
	pid[2] = m_kd;
}

#pragma endregion PID_Controller
//
template class icalab::FcImpedance1<float>;
template class icalab::FcImpedance1<double>;
template class icalab::FcPID<float>;
template class icalab::FcPID<double>;

// 剛性估測器有出一些問題需要進行微幅的修改：
//template<class T> void icalab::FcImpedance1<T>::StiffnessEstimator(T fe_tcp, T x, T xe) {
//	T&& upper_bound = ke_hat_init * ((T)1.0 + ke_vary_ratio);
//	T&& lower_bound = ke_hat_init * ((T)1.0 + ke_vary_ratio);
//	fe_tcp -= m_fc_prePress;
//	switch (optimizer) {
//	case GRADIENT_DESCENT: {// Gradient Descent:
//		T&& x_minus_xe = x - xe;
//		gradient = x_minus_xe * (-fe_tcp + ke_hat * x_minus_xe);
//		ke_hat -= ke_lr * gradient;
//		break;
//	}
//	case ADAM: { // Adaptive Moment Estimation (reference: https://arxiv.org/abs/1412.6980)
//		adam_m = adam_betta1 * adam_m + (1 - adam_betta1) * gradient;
//		adam_v = adam_betta2 * adam_m + (1 - adam_betta2) * gradient * gradient;
//		ke_hat -= ke_lr * ((adam_m / (1 - adam_betta1)) / (sqrt(adam_v / (1 - adam_betta2)) + (T)0.00000001));
//		break;
//	}
//	case AVERAGE_GRAD: { // Averaged Gradient Descent
//		static unsigned int k = 0;
//		static T integral = (T)0.0;
//		T&& x_minus_xe = x - xe;
//		integral += x_minus_xe * (-fe_tcp + ke_hat * x_minus_xe);
//		// averaging:
//		if (++k > grad_integral_size) {
//			gradient = integral / (T)100.0;
//			integral = (T)0.0;
//			k = 0;
//		}
//		ke_hat -= ke_lr * gradient;
//
//		break;
//	}
//	}
//	this->invKm_plus_invKe = (T)1.0 / km + (T)1.0 / ke_hat;
//	this->invKmKe = ke_hat / km;
//}
////
//template<class T> void icalab::FcImpedance1<T>::setAverageGradStiffnessEstimator(unsigned int integral_size) {
//	this->grad_integral_size = integral_size;
//}
////
//template<class T> void icalab::FcImpedance1<T>::setAdamStiffnessEstimator(T betta1, T betta2) {
//	this->adam_betta1 = betta1;
//	this->adam_betta2 = betta2;
//}
////
//template<class T> void icalab::FcImpedance1<T>::resetStiffnessEstimator(void) {
//	this->grad_integral_size = 100;
//	this->ke_hat = this->ke_hat_init;
//	this->optimizer = GRADIENT_DESCENT;
//	this->gradient = (T)0.0;
//	this->adam_m = (T)0.0;
//	this->adam_v = (T)0.0;
//	this->adam_betta1 = (T)0.9;
//	this->adam_betta2 = (T)0.999;
//}
////
//template<class T> void icalab::FcImpedance1<T>::setInitialStiffness(T ke_init) {
//	this->ke_hat_init = ke_init;
//}
////
//template<class T> void icalab::FcImpedance1<T>::setLearningRate(T lr) {
//	this->ke_lr = lr;
//}