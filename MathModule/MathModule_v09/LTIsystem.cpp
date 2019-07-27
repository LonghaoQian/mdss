#include "stdafx.h"
#include "LTIsystem.h"

LTIsystem::LTIsystem()
{}

LTIsystem::LTIsystem(const LTIParameter& parameter, const LTIInitialCondition& IC)
{
	state = IC.X_0;// assign the initial condition to the state
	system_info.system_type = LTI;
	A = parameter.A;
	B = parameter.B;
	C = parameter.C;
	D = parameter.D;
	// determine whether D is a zero matrix
	if (D.norm() < 0.0000000001)// very small norm
	{
		system_info.DIRECT_FEED_THROUGH = false;
	}
	else {
		system_info.DIRECT_FEED_THROUGH = true;
	}
	// determine the size of the system
	system_info.num_of_continuous_states = A.rows();
	system_info.num_of_inputs = B.cols();
	system_info.num_of_outputs = C.rows();
	// check the compatibility of the system matrices:
	system_info.system_parameter_ok = true;
	if (A.rows() != B.rows())
	{
		system_info.system_parameter_ok = false;
	}

	if (A.rows() != A.cols())
	{
		system_info.system_parameter_ok = false;
	}

	if (A.rows() !=  C.cols())
	{
		system_info.system_parameter_ok = false;
	}

	if (C.rows() != D.rows())
	{
		system_info.system_parameter_ok = false;
	}

	if (system_info.num_of_inputs != D.cols())
	{
		system_info.system_parameter_ok = false;
	}

	if (system_info.system_parameter_ok == true)
	{
		ready_to_run = true;
		// initialize state memeory
		state.resize(system_info.num_of_continuous_states);
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
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

void LTIsystem::OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output)
{
	// y = Cx + Du
	output = C * state + D * input;
}

void LTIsystem::IncrementState()
{
	state += solver_buffer_state_increment1;
}

VectorXd LTIsystem::GetState()
{
	return state;
}

void LTIsystem::DisplayParameters()
{
	cout << "---------------------" << endl;
	cout<< "LTI system parameter:"<<endl;
	cout << "A = " << endl << A << endl;
	cout << "----" << endl;
	cout<< "B = " <<endl<< B << endl;
	cout << "----" << endl;
	cout<< "C = " <<endl << C << endl;
	cout << "----" << endl;
	cout<< "D = " <<endl<< D << endl;
}

void LTIsystem::DisplayInitialCondition()
{
	cout << "---------------------" << endl;
	cout << "LTI system initial condition X_0:" << endl;
	cout << "The initial condition is:  " << endl;
	cout << state << endl;
}

