#pragma once
#include "Subsystem.h"
#include <iostream>
#define READYTOGO 0;
#define STATE_INPUT_MISMATCH 1;
#define STATE_OUTPUT_MISMATCH 2;
#define INPUT_OUTPUT_MISMATCH 3;
struct LTIParameter{
	MatrixXd A;
	MatrixXd B;
	MatrixXd C; 
	MatrixXd D;
};

struct LTIInitialCondition {
	VectorXd X_0;
};
using namespace std;
class LTIsystem :
	public Subsystem
{
private:
	// parameters
	MatrixXd A, B, C, D;
public:
	LTIsystem();
	LTIsystem(const LTIParameter& parameter, const LTIInitialCondition& IC);
	void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
	void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
	void IncrementState();
	void DisplayParameters();
	void DisplayInitialCondition();
	~LTIsystem();
};

