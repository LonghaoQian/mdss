#pragma once
#include "Subsystem.h"
#define READYTOGO 0;
#define STATE_INPUT_MISMATCH 1;
#define STATE_OUTPUT_MISMATCH 2;
#define INPUT_OUTPUT_MISMATCH 3;
class LTIsystem :
	public Subsystem
{
private:
	// parameters
	MatrixXd A, B, C, D;
public:
	LTIsystem();
	LTIsystem(const MatrixXd& A_, const MatrixXd& B_, const MatrixXd& C_, const MatrixXd& D_);
	void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
	void OutputEquation(const VectorXd& state, const VectorXd& input, VectorXd& output);
	void LoadInitialCondition(const VectorXd& initial_condition);
	void LoadParameters(const VectorXd& parameter_list);
	~LTIsystem();
};

