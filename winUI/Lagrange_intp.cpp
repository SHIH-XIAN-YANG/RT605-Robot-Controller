#include "Lagrange_intp.h"

Lagrange_intp::Lagrange_intp(int max_iter, double max_feature, double min_feature, double max_gain, double min_gain, double rt605_ts, int intp_pointCount){
	this->max_iter = max_iter;
	this->iter = 0;

	this->max_feature = max_feature;
	this->min_feature = min_feature;

	this->max_gain = max_gain;
	this->min_gain = min_gain;

	this->intp_pointCount = intp_pointCount;

	this->gain_type = TuneGainType::kvp;
	this->loop_type = TuneLoopType::vel;

	this->rt605_ts = rt605_ts;
}

Lagrange_intp::~Lagrange_intp() {}


double Lagrange_intp::intp(const std::vector<double>&x_data, const std::vector<double>&y_data, TuneLoopType loop_type, TuneGainType gain_type) {
	/* Implement Lagrange interpolation and compute the next optimal gain */
	double opt_gain = -1; // next optimal gain
	if (iter == 0) {
		opt_gain = min_gain + features[0] * (max_gain - min_gain) / (max_feature - min_feature);
	}
	else {
		double min_y = DBL_MAX;
		double x, y, term;

		size_t i, j;
		size_t data_size = x_data.size();

		for (auto idx = 0; idx < intp_pointCount; idx++) {
			x = min_gain + idx * (max_gain - min_gain) / intp_pointCount;
			y = 0.0;

			for (i = 0; i < data_size; i++) {
				term = features[i];
				for (j = 0; j < data_size; j++) {
					if (i != j && x_data[i] != x_data[j]) {
						term = term * (x - x_data[j]) / (x_data[i] - x_data[j]);
					}
				}
				y += term;
			}

			if (abs(y) < min_y) {
				min_y = abs(y);
				opt_gain = x;
			}
		}
	}

	if (gain_type == TuneGainType::kpp) {
		kpp.push_back(opt_gain);
	}
	else if (gain_type == TuneGainType::kvi) {
		kvi.push_back(opt_gain);
	}
	else if (gain_type == TuneGainType::kvp) {
		kvp.push_back(opt_gain);
	}

	iter++;

	opt_gain = fmin(max_gain, fmax(min_gain, opt_gain));

	return opt_gain;
}

double Lagrange_intp::compute_feature(const std::vector<double>& joint_data_act,
	const std::vector<double>& joint_data_ref, FeatureType feature_type) {
	/* compute the feature from joint_data_act and joint_data_ref */
	double feature=0.0;
	double tau = 2;
	if (feature_type == FeatureType::magnitude) {
		feature = *std::max_element(joint_data_ref.begin(), joint_data_ref.end()) - *std::max_element(joint_data_act.begin(), joint_data_act.end());
	}
	else if (feature_type == FeatureType::phase) {
		feature = rt605_ts * static_cast<double>(find_closest_index(joint_data_act, joint_data_ref[0]) - find_closest_index(joint_data_ref, joint_data_ref[0]));
	}
	else if(feature_type == FeatureType::mag_phase) {
		double magnitude = *std::max_element(joint_data_ref.begin(), joint_data_ref.end()) - *std::max_element(joint_data_act.begin(), joint_data_act.end());
		double phase_shift = rt605_ts * static_cast<double>(find_closest_index(joint_data_act, joint_data_ref[0]) - find_closest_index(joint_data_ref, joint_data_ref[0]));;

		feature = magnitude * exp(phase_shift / tau);
	}

	return feature;
}

int Lagrange_intp::find_closest_index(const std::vector<double>& joint_data_act,
	double target) {
	/*
    * find the closest index of trajectory's  to the target
    */
	double min_diff = DBL_MAX;
	int closest_idx = -1;

	int quarter_len = joint_data_act.size() / 4;
	int three_quarter_len = (joint_data_act.size() * 3) / 4;
	double diff;
	for (auto idx = quarter_len; idx < three_quarter_len; idx++) {
		diff = abs(joint_data_act[idx] - target);
		if (diff < min_diff) {
			min_diff = diff;
			closest_idx = idx;
		}
	}
	 
	return closest_idx;
}

//std::vector<double> Lagrange_intp::getFeatures(void) {
//	std::vector<double> result_features;
//
//	result_features.assign(features.begin(), features.end());
//
//	return result_features;
//}
//
//std::vector<double> Lagrange_intp::getGain(TuneGainType gain_type) {
//	std::vector<double> result_gain;
//
//	if (gain_type == TuneGainType::kpp) {
//		result_gain.assign(kpp.begin(), kpp.end());
//	}
//	else if (gain_type == TuneGainType::kvi) {
//		result_gain.assign(kvi.begin(), kvi.end());
//	}
//	else if (gain_type == TuneGainType::kvp) {
//		result_gain.assign(kvp.begin(), kvp.end());
//	}
//	return result_gain;
//}

void Lagrange_intp::export_log(void) {
	/* export log to json or csv*/

}