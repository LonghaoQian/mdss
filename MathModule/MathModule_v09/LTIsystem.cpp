#include "stdafx.h"
#include "LTIsystem.h"

LTIsystem::LTIsystem()
{}

LTIsystem::LTIsystem(const MatrixXd& A_, const MatrixXd& B_, const MatrixXd& C_, const MatrixXd& D_)
{
	A = A_;
	B = B_;
	C = C_;
	D = D_;
	// determine the size of the system
	num_of_continuous_states = A.rows();
	num_of_inputs = B.cols();
	num_of_outputs = C.rows();

	// check the compatibility of the system matrices:
	system_info = 0;
	if (A.rows() != B.rows())
	{
		system_info = STATE_INPUT_MISMATCH;
	}

	if (A.rows() != A.cols())
	{
		system_info = STATE_INPUT_MISMATCH;
	}

	if (A.rows() !=  C.cols())
	{
		system_info = STATE_OUTPUT_MISMATCH;
	}

	if (C.rows() != D.rows())
	{
		system_info = INPUT_OUTPUT_MISMATCH;
	}

	if (num_of_inputs != D.cols())
	{
		system_info = INPUT_OUTPUT_MISMATCH;
	}

	if (system_info == 0)
	{
		ready_to_run = true;
		// initialize state memeory
		state.resize(num_of_continuous_states);
		output.resize(num_of_outputs);
	}
	else {
		ready_to_run = false;
	}
	

}


LTIsystem::~LTIsystem()
{
}


void LTIsystem::DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative)
{
	// x_dot  = A x + B u
	derivative = A * state + B * input;
}

void LTIsystem::OutputEquation(const VectorXd& state, const VectorXd& input, VectorXd& output)
{
	// y = Cx + Du
	output = C * state + D * input;
}

void LTIsystem::LoadInitialCondition(const VectorXd& initial_condition)
{
}

void LTIsystem::LoadParameters(const VectorXd& parameter_list)
{
}

