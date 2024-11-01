#include "Path_Generator.h"

Path_Generator::Path_Generator(arma::Row<float> p0, arma::Row<float> ptg, unsigned int knot_num, float IC, float BC){
	// setting up the essential attributes
	_Pos_0 = p0;
	_Pos_tg = ptg;
	_IC = IC;
	_BC = BC;
	_knot_num = knot_num;
}
bool Path_Generator::Set_Kernel0(std::vector<float> group0){
	_F_H = group0.at(0)  ;
	_Acc_max = group0.at(1);
	_ts = group0.at(2);
	_t0 = group0.at(3);
	return true;
}
void Path_Generator:: SLINE1_AdjustFeedrate(float& ts, float& t0, float& t_acc, float& acc, float& F_L, float& F_H, float& L, float& tol) {
	// Sigmoid-shape LINE PTP feedrate adjust process
	// To find the maximum allowable feedrate F_star
	float acc_trapz{ (float)((double)acc / acc_beta) }, t_acc_star{ F_H / acc_trapz }, F_star{ F_H };
	float F_ceil{ F_H }, F_floor{ F_L };
	t_acc_star = F_star / acc_trapz;
	t_acc = t_acc_star; // load the estimated t_acc in the first time
	unsigned int n = 1 + (t_acc_star - t0) / ts; // data length
	dataF8 time = arma::conv_to<dataF8>::from(arma::linspace(t0, t_acc_star, n)); time = arma::trunc((1 / ts) * time) * ts;
	dataF8 vel_star(n); 
	vel_star.at(0) = F_L;
	vel_star.at(n - 1) = F_H;
	interp1 vel_gen(ts, t_acc_star, F_L, F_star, "sigmoid");
	dataF8 A(1); float B{ 0 }; int ii{ 0 }; bool condition{ false };
	dataF8 L_star{ 0 };
	for (;;) {
		//std::cout << ii  << "), B = " << B  << " , A = " << A.at(0) << std::endl;
		std::generate(vel_star.begin() + 1, vel_star.end() - 1, vel_gen);
		A = 2 * arma::trapz(time, vel_star); L_star = A.at(0);
		// compute the area of the acc-region and the dec-region
		if (A.at(0) <= L) {
			if (F_star == F_H) { t_acc = t_acc_star;  condition = true; goto goal; } // complete
			else if (F_star != F_H) {// find the maximum feedrate within the tolerance (the feedrate is not sufficient quick)
				F_floor = F_star; B = abs(L - A.at(0));
				if (B > tol) {
					F_star = (F_star + F_ceil) / 2; t_acc_star = F_star / acc_trapz;
				} // adjust F and t_acc
				else if (B <= tol) {
					F_H = F_star; t_acc = t_acc_star; condition = true; goto goal;
				} // complete
			}
		}
		else if (A.at(0) > L) { // adjust F and t_acc
			F_ceil = F_star;
			F_star = (F_star + F_floor) / 2;
			t_acc_star = F_star / acc_trapz;
		}
		vel_gen.setParameters(ptr_T, t_acc_star); vel_gen.setParameters(ptr_ytg, F_star);
		n = 1 + (t_acc_star - t0) / ts; // reset the data length
		time.reset(); time = arma::conv_to<dataF8>::from(arma::linspace(t0, t_acc_star, n)); time = arma::trunc(100 * time) / 100;// reset the time_acc series
		vel_star.reset(); vel_star.resize(n); vel_star.at(0) = F_L; vel_star.at(n - 1) = F_star; // reset the velocity_acc series
		//system("cls"); std::cout << time << std::endl;
		++ii;
	goal:
		if (condition) { t_acc = round((1 / ts) * t_acc_star) * _ts; break; };
	}
	std::cout << "Adjust process had been executed for " << ii << " loop." << std::endl;
} // 2020/08/26 -B.R.Tseng
PVT_PATH Path_Generator::S_LINE3(float tol) {
	float Feedrate_L{ _IC }, Feedrat_H{ _F_H };
	// step1: determine the t_acc and t_const
	// compute the smallest distance between initial position and target position
	_L = arma::norm(_Pos_tg - _Pos_0, 2);
	SLINE1_AdjustFeedrate(_ts, _t0, _t_acc, _Acc_max, _IC, _F_H, _L, tol);
	// end adjust process
		_t_const = round((1 / _ts) * (_L / _F_H)) * _ts;
		unsigned int n1{ (unsigned long)((_t_acc - _t0) / _ts) };
		unsigned int n2; n2 = _t_const > _t_acc ? (unsigned int)((_t_const -_t_acc - _t0) / _ts) : 0;
		unsigned int n3 = n1;
		unsigned int n = 1 + n1 + n2 + n3; // including origin point
		// step2: generate the movement path
		It_gen time_generate("time");
		dataF8 time = time_generate.incremental_series(_t0, (n1 + n2 + n3 + 1), _ts);  // generate time series
		dataF8 vel(n); //std::cout << vel.size() << "\n" << time.size() << std::endl;
		// generate the Acc. segment of velocity path
		vel.at(0) = _IC; vel.at(n1) = _F_H;
		interp1 vel_gen(_ts, _t_acc, _IC, _F_H, "sigmoid");

		std::generate(vel.begin() + 1, vel.begin() + n1, vel_gen);  //std::cout << "\n[acc]:\n " <<vel << std::endl;
		// generate the constant velocity path
		if (n2 != 0) { // check it if there exist constant-velocity region
			vel_gen.setParameters(ptr_y0, _F_H); vel_gen.setParameters(ptr_ytg, _F_H);
			vel_gen.setType("linear"); // reset the interpolator
			std::generate(vel.begin() + n1 + 1, vel.begin() + n1 + n2, vel_gen); //std::cout << "\n[const]: \n" << vel << std::endl;
			// generate the Dec. segment of velocity path
			vel_gen.setParameters(ptr_T, _t_acc); vel_gen.setParameters(ptr_y0, _F_H);
			vel_gen.setParameters(ptr_ytg, 0); vel_gen.setType("sigmoid");
			if (n2 != 0) {
				vel.at(n1 + n2) = _F_H;
				vel.at(n1 + n2 + n3) = 0;
				std::generate(vel.begin() + n1 + n2, vel.end() - 1, vel_gen);
			}
			else {
				vel.at(n1 + n2 + n3) = 0; //std::cout << n1 + n2 + n3 << "  " << vel.at(1228);
				std::generate(vel.begin() + n1 + 1, vel.end() - 1, vel_gen);
			}
		}
		_tf = time.at(time.size() - 1);
		// estimate the position data points 
		dataF8 pos(n), posX(n), posY(n), posZ(n);
		dataF8 velX(n), velY(n), velZ(n);
		arma::Row<float> path_vector = _Pos_tg - _Pos_0;
		float L1 = sqrt((_Pos_tg(0) - _Pos_0(0))* (_Pos_tg(0) - _Pos_0(0)) + (_Pos_tg(1) - _Pos_0(1))* (_Pos_tg(1) - _Pos_0(1)));
		float L2 = sqrt(_L*_L - L1*L1);
		float phiZ = (float)atan2(L2, L1);
		float thetaXY = (float)atan2(path_vector.at(1), path_vector.at(0));
		It_gen pos_gen(time, vel,"inter_trapz");
		pos(0) = 0; pos(n-1) = _L;
		std::generate(pos.begin()+1, pos.end()-1, pos_gen);
		dataF8 posXY = pos * cos(phiZ);
		posX = _Pos_0.at(0) + posXY * cos(thetaXY);
		posY = _Pos_0.at(1) + posXY * sin(thetaXY);
		posZ = _Pos_0.at(2) + pos * sin(phiZ);
		dataF8 velXY = vel * cos(phiZ);
		velX = _IC + velXY * cos(thetaXY);
		velY = _IC + velXY * sin(thetaXY);
		velZ = _IC + vel * sin(phiZ);
		//std::cout << vel << std::endl;
		/*dataset path_vel = {"vel(t)", arma::conv_to<dataDOU>::from(vel)};
		dataset path_velX = {"vel"};
		dataset path_time = {"time", arma::conv_to<dataDOU>::from(time)};
		dataset path_pos = { "time", arma::conv_to<dataDOU>::from(pos) };
		datafile file1 = { path_time, path_vel, path_pos };
		if(export_datafile("D:\\test02.csv", file1))
			std::cout << "complete" << std::endl;*/
	//********************** return values *************************************************
	// unit: time => second; position => mm; velocity => mm/sec
	// (1) pair.first: return motion parameters:
	// (2) pair.second: return PTV-datas (std::vector<float>)
	//**************************************************************************************
		std::vector<float> parameters; parameters = { _t0, _tf, _ts, (float)n, _F_H, _L };
		std::vector<dataF8> PVT_path(9, dataF8(n)); PVT_path = {time, pos, vel, posX, posY, posZ, velX, velY, velZ };
		return  { parameters, PVT_path };
}
