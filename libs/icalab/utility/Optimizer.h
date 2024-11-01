#ifndef __OPTIMIZER__H
#define __OPTIMIZER__H
#include<iostream>
#include<vector>
#include<string>
#include<algorithm>
#include<random>
#include<functional> // for std::bind 
#include<chrono> // for generate the random seed
#include"armadillo"
//------------- Macro and Typedefine ------------------------
// typedefine:

// Optimizer::optimization type:
#define pso 1
// Optimizer::initialization type
#define uniform 101 
#define random 102
// Optimizer::objective tendency:
#define min false // default: minimize
#define max true // maximize
// Swarm::Parameters 
#define PopulationSize 201
#define Dimension 202
#define Gfitness 204
#define BC 205
//--------------------------------------------------
// Particle Swarm Optimization algorithm

class Agent {
	protected:
		unsigned short _Dim; // the dimension of the solution
		float _Pfitness; // personal fitness
	public:
		std::vector<float> _sol; // solution vector
		Agent(unsigned short dim){
			_Dim = dim;
			_sol.reserve(_Dim); _sol.resize(_Dim);
		}
		Agent (void) {}
		std::vector<float> getSol(void);
		void setDimension(unsigned short dim);
		//int a{99};
};

class Swarm  {
	protected:
		unsigned short _PopulationSize;
		unsigned short _Dim; // the dimension of the solution
		std::vector<std::pair<float, float>> _BC; // boundary condition
		std::vector<float> _gbest; // the best solution among the group
		float _Gfitness; // the best fitness of the grouping-best solution
		
	public:
		std::vector<Agent> _swarm; // generate the swarm
		Swarm(unsigned short size, unsigned short dim){
			_PopulationSize = size; _Dim = dim;
			_swarm.reserve(_PopulationSize); _swarm.resize(_PopulationSize);
			for(unsigned short i=0;i<size;++i)
			_swarm.at(i).setDimension(dim);
		}
		unsigned short getPrt(unsigned short prt); // asking for parameters*/
		void setBC(std::vector<std::pair<float, float>>& bc);
		std::pair<float, float> getBC(unsigned short index); // get the BC
};

class Optimizer {
protected:
	unsigned short _Type{ 0 };
	bool _Objective{ 0 }; // objective, e.g. minimize, maximize
	unsigned short _IterationNum; // amount of iteration steps
// Process record:
	std::vector<float> _fitness; // record the best fitness of group in each iteration
public:
	bool initialize(Swarm& gp, unsigned short method);

};

#endif// the latest version: 2020/10/06 -B.R.Tseng
//------------------------------------------------------------------------
// 2020/10/06: Adding PSO algorithm.