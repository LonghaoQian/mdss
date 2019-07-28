#pragma once
#include "Subsystem.h"
#include <iostream>
#define READYTOGO 0
#define GAIN_MATRIX 1 
#define GAIN_ElEMENT_WISE 2
#define GAIN_SCALER 3
struct Gainparameter {
	MatrixXd K;
	int type;
	int num_of_inputs;
};
using namespace std;
class Math_Gain :
	public Subsystem
{
private:
	int Gain_Type;
	MatrixXd K;
public:
	Math_Gain(const Gainparameter& _gainparameter);
	void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
	/*-----Only output function is used, the rest is set to empty functions just to keep the subsystem format-------------------*/
	void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& temp_derivative);
	void IncrementState();
	VectorXd GetState();
	void DisplayParameters();
	void DisplayInitialCondition();
	~Math_Gain();
};

