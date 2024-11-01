/*
*  -	     Build: 2022/06/11, B.R. Tseng
*  - The last edit: 2023/05/17, B.R. Tseng
*/
#ifndef __FILTER_H__
#define __FILTER_H__
#include "eigen-3.4.0/Eigen/Dense"
//#include"../lib/eigen-3.4.0/Eigen/Dense"
#include<stdio.h>
//#include<exception>
#include<utility>
#include<string.h>
/* Note:
* 由於 ButterWorth 的極零點之計算還載推導與整理，故暫時使用線上的計算器 (Digital Butterworth filter calculator: https://www.meme.net.au/butterworth.html)
* 來幫忙計算出 極零點，所以下方之 example 中設定 sampling rate 與 cuttoff frequency 暫時是沒有作用的，其中只有設定濾坡器的街數是有意義的。
*/
namespace filter{
	class IIR_Base {
	protected:
		Eigen::MatrixXd m_kernel_zeros, m_kernel_poles;
		Eigen::MatrixXd m_Y_buf, m_X_buf;
		unsigned short m_filter_order;

	public:

		inline void initialize(void) {
			m_X_buf.setZero();
			m_Y_buf.setZero();
		}
		inline void initialize(double x_0, double y_0) {
			m_X_buf.setConstant(x_0);
			m_Y_buf.setConstant(y_0);
		}
		inline void BuildFilter(double* zeros, double* poles) {
			for (unsigned short i = 0; i < m_filter_order; ++i) {
				m_kernel_zeros(0, i) = zeros[i];
				m_kernel_poles(0, i) = poles[i];
			}
			m_kernel_poles(0, m_filter_order) = poles[m_filter_order];
		}
		//
		inline void showPoles() {
			for (unsigned short i = 0; i < m_filter_order + 1; ++i)
				printf("%lf,  ", m_kernel_poles(i));
			printf("\n");
		}
		//
		inline void showZeros() {
			for (unsigned short i = 0; i < m_filter_order; ++i)
				printf("%lf,  ", m_kernel_zeros(i));
			printf("\n");
		}
		//
		//inline double operator()(const double& x_i) {
		//	memmove_s(m_X_buf.data() + 1, (m_filter_order) * sizeof(double), m_X_buf.data(), (m_filter_order) * sizeof(double));
		//	m_X_buf(0, 0) = x_i;
		//	double&& y_i = ((m_kernel_poles * m_X_buf) + (m_kernel_zeros * m_Y_buf)).value();
		//	memmove_s(m_Y_buf.data() + 1, (m_filter_order - 1) * sizeof(double), m_Y_buf.data(), (m_filter_order - 1) * sizeof(double));
		//	m_Y_buf(0, 0) = y_i;
		//	return y_i;
		//}
		inline double operator()(double x_i) {
			memmove_s(m_X_buf.data() + 1, (m_filter_order) * sizeof(double), m_X_buf.data(), (m_filter_order) * sizeof(double));
			m_X_buf(0, 0) = x_i;
			double&& y_i = ((m_kernel_poles * m_X_buf) + (m_kernel_zeros * m_Y_buf)).value();
			memmove_s(m_Y_buf.data() + 1, (m_filter_order - 1) * sizeof(double), m_Y_buf.data(), (m_filter_order - 1) * sizeof(double));
			m_Y_buf(0, 0) = y_i;
			return y_i;
		}
		
		//
	};
	//
	class FIR_Base {
	protected:
		Eigen::MatrixXd m_kernel_poles;
		Eigen::MatrixXd m_X_buf;
		unsigned short m_filter_order;

	public:

		inline void initialize(void) {
			m_X_buf.setZero();
		}
		inline void initialize(double x_0, double y_0) {
			m_X_buf.setConstant(x_0);
		}
		inline void BuildFilter(double* poles) {
			for (unsigned short i = 0; i < m_filter_order; ++i) {
				m_kernel_poles(0, i) = poles[i];
			}
			m_kernel_poles(0, m_filter_order) = poles[m_filter_order];
		}
		//
		inline void showPoles() {
			for (unsigned short i = 0; i < m_filter_order + 1; ++i)
				printf("%lf,  ", m_kernel_poles(i));
			printf("\n");
		}
		//
		//inline double operator()(const double& x_i) {
		//	memmove_s(m_X_buf.data() + 1, (m_filter_order) * sizeof(double), m_X_buf.data(), (m_filter_order) * sizeof(double));
		//	m_X_buf(0, 0) = x_i;
		//	return (m_kernel_poles * m_X_buf).value();
		//}
		inline double operator()(double x_i) {
			memmove_s(m_X_buf.data() + 1, (m_filter_order) * sizeof(double), m_X_buf.data(), (m_filter_order) * sizeof(double));
			m_X_buf(0, 0) = x_i;
			return (m_kernel_poles * m_X_buf).value();
		}
		//
	};
	//
	class Butterworth : public IIR_Base
	{
		/* 
		* - Digital Butterworth filter calculator: https://www.meme.net.au/butterworth.html
		* 
		*/
	private:
		double m_sampling_rate; // unit: Hz
		double m_cutoff_frequency; // unit: Hz
	public:
		inline Butterworth(void) {}
		inline Butterworth(unsigned short _order, double _samplingRate, double _cutoffFrequency) {
			setup(_order, _samplingRate, _cutoffFrequency);
		}
		inline void setup(unsigned short _order, double _samplingRate, double _cutoffFrequency){
			m_filter_order = _order;
			m_sampling_rate = _samplingRate;
			m_cutoff_frequency = _cutoffFrequency;
			m_kernel_zeros.resize(1, _order);
			m_kernel_poles.resize(1, _order + 1);
			m_Y_buf.resize(_order, 1);
			m_X_buf.resize(_order + 1, 1);
			initialize();
		}
	};
	//
	class MovingAverage : public FIR_Base {
	public:
		inline MovingAverage(void) {}
		inline ~MovingAverage(void) {}
		inline void setup(unsigned short _order){
			m_filter_order = _order;
			m_kernel_poles.resize(1, _order + 1);
			m_X_buf.resize(_order + 1, 1);
			initialize();
			double* poles_tmp = static_cast<double*>(malloc(sizeof(double) * (_order + 1)));
			for (unsigned short i = 0; i < _order + 1; ++i)
				poles_tmp[i] = 1.0 / static_cast<double>(_order + 1);
			BuildFilter(poles_tmp);
			delete[] poles_tmp;
		}
	};
}
#endif
/*
// generate a noisy sine sequence:
double samplingRate = 1000.0;
double ts = 1.0/samplingRate; // define sampling time ts = 1/fs;
double x[1000]; // 1 second
for(int i = 0; i<1000; ++i){
	double t = i*ts;
	x[i] = 1.0*sin(2*3.14/0.5*t);
	// add noise:
	x[i] += 0.2*sin(10.0*2*3.14/0.5*t) + 0.1*cos(5.0*2*3.14/0.5*t) + 0.3*sin(21.0*2*3.14/0.5*t);
}
// Example1: (ButterWorth Filter, 3-order, fc = 2.4 Hz, fs = 1 KHz)
unsigned short order = 3;
double cutoffFrequency = 2.4;

Butterworth f1;
f1.setup(order, samplingRate, cutoffFrequency);
double den = 18805071.216;
double zeros[3] = { 56131640.527 / den, -55850201.461 / den, 18523624.151 / den };
double poles[4] = { 1.0 / den , 3.0 / den , 3.0 / den , 1.0 / den };
f1.BuildFilter(zeros, poles);

double y1[1000];
// filtering:
for(int i = 0; i<1000; ++i)
	y1[i] = f1(x[i]);

// Example2: (Moving Average filter, 50-order)
unsigned short order = 50;

MovingAverage f2;
f2.setup(order);

double y2[1000];
// filtering:
for(int i = 0; i<1000; ++i)
	y2[i] = f2(x[i]);
*/