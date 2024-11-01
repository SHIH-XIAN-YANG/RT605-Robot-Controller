#ifndef __PATH_GENERATOR_H__
#define __PATH_GENERATOR_H__
#include "Interp1.h"
#include "Iterative_tool.h"
#include<algorithm> // for std::generate
#include"Data_Processing.h"
#include<utility>
#define acc_beta 6.285984949405756
// shape define: position trajectory
//#define sigmoid 191
//#define cubic 193
//#define line 194
typedef std::pair<std::vector<float>, std::vector<dataF8>> PVT_PATH;
// 2020/08/27 -B.R.Tseng
class Path_Generator {
// Path generate Module
//
	private:
	//---------------- Attrubutes ----------------
	//unsigned short _shape {0}; // main velocity trajectory shpae or Acc. trajectory shpae
	arma::Row<float> _Pos_0, _Pos_tg;
	unsigned int _knot_num {0}; // numbers of via points
	float _IC {0}; // initial condition
	float _BC {0}; // boundary condition (e.g. the velocity at the final point)
	//---------------- Kernel ----------------
	// need to be loaded or be defined by user 
	// kernel group0
	float _F_H; // maximum feedrate
	float _Acc_max; // maximum acceleration
	float _ts; // sampling time or step time
	float _t0; // initial time
	
	// preserved kernel group1
	//std::vector<unsigned int> _limit01; 
	// preserved kernel group2
	//std::vector<double> _limit02; 
	// group A. operation parameters, after estimating process 
	float _t_acc;
	float _L; // total movement distance
	float _t_const;
	float _tf;
	//PVT_PATH _PATH;
	//---------------- Current state ----------------
	 
	// privat API's
	void SLINE1_AdjustFeedrate(float& ts, float& t0, float& t_acc, float& acc, float& F_L, float& F_H, float& L, float& tol); // only for S-LINE function
	public:
	//---------------- Constructor ----------------
	Path_Generator(arma::Row<float> p0, arma::Row<float> ptg, unsigned int knot_num, float IC, float BC); //2020/09/03 -B.R.Tseng
	
	//---------------- API's ----------------
	bool Set_Kernel0(std::vector<float> group0); //2020/09/03 -B.R.Tseng
	PVT_PATH  S_LINE3(float tol);
};
#endif // the latest version: 2020/09/03 -B.R.Tseng
//--------------------- Reversion -----------------------------------------
// 
//------------------------------------------------------------------------