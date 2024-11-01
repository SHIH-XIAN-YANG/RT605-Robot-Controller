#include"Optimizer.h"

Optimizer::Optimizer(std::string Type){
	if( !Type.compare("PSO") || !Type.compare("pso"))
		_Type = pso;
}
// the latest version: 2020/10/06 -B.R.Tseng
//------------------------------------------------------------------------