#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <float.h>

enum class TuneLoopType {
	vel,
	pos
};

enum class TuneGainType {
	kpp,
	kvp,
	kvi
};

enum class FeatureType {
	phase,
	magnitude,
	mag_phase
};

class Lagrange_intp
{
public:
	Lagrange_intp(int, double, double, double, double, double, int);
	~Lagrange_intp();
	double intp(const std::vector<double>&, const std::vector<double>&, TuneLoopType, TuneGainType);
	void export_log(void);
	double compute_feature(const std::vector<double>&,
		const std::vector<double>&, FeatureType);

	void set_gain_limit(double min_gain, double max_gain) { this->max_gain = max_gain; this->min_gain = min_gain; }
	void set_feature_limit(double min_feature, double max_feature) { this->max_feature = max_feature; this->min_feature = min_feature; }

	//std::vector<double> getFeatures(void);
	//std::vector<double> getGain(TuneGainType);

private:
	//std::vector<std::vector<double>> joint_data_ref;
	//std::vector<std::vector<double>> joint_data_act;

	int iter;
	int max_iter;
	double rt605_ts;

	std::vector<double> features;
	std::vector<double> kpp;
	std::vector<double> kvp;
	std::vector<double> kvi;

	double max_feature;
	double min_feature;
	double max_gain;
	double min_gain;
	int intp_pointCount;
	TuneLoopType loop_type;
	TuneGainType gain_type;
	
	int find_closest_index(const std::vector<double>&,
		double);
};

