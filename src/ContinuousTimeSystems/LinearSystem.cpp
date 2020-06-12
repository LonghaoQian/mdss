#include "pch.h"
#include "LinearSystem.h"
namespace linearsystem {
	LTIsystem::LTIsystem()
	{}

	LTIsystem::LTIsystem(const LTIParameter& parameter, const LTIInitialCondition& IC)
	{
		state = IC.X_0;// assign the initial condition to the state
		system_info.type = continous_LTI;
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

		if (A.rows() != C.cols())
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
	void LTIsystem::DisplayParameters()
	{
		cout << "---------------------" << endl;
		cout << "LTI system parameter:" << endl;
		cout << "A = " << endl << A << endl;
		cout << "----" << endl;
		cout << "B = " << endl << B << endl;
		cout << "----" << endl;
		cout << "C = " << endl << C << endl;
		cout << "----" << endl;
		cout << "D = " << endl << D << endl;
	}

	void LTIsystem::DisplayInitialCondition()
	{
		cout << "---------------------" << endl;
		cout << "LTI system initial condition X_0:" << endl;
		cout << "The initial condition is:  " << endl;
		cout << state << endl;
	}

	Integrator::Integrator()
	{
	}

	Integrator::Integrator(const IntegratorParameter & parameter_, const IntegratorInitialCondition & IC)
	{
		state = IC.X_0;// assign the initial condition to the state
		parameter = parameter_;
		// sets the system type and category 
		system_info.category = LINEARSYSTEM;
		system_info.type = continous_INTEGRATOR;
		// set ready_to_run to true since there is not parameter check for this subsystem
		ready_to_run = true;
		// determine the size of the system
		system_info.num_of_continuous_states = parameter.num_of_channels;
		system_info.num_of_inputs		     = parameter.num_of_channels;
		system_info.num_of_outputs           = parameter.num_of_channels;
		// check the compatibility of the system matrices:
		system_info.system_parameter_ok = true;
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = false;
		// initialize state memeory
		state.resize(system_info.num_of_continuous_states);
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
	}

	void Integrator::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// x_dot  = u
		derivative = input;
	}

	void Integrator::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		// y = x
		output = state;
	}

	void Integrator::IncrementState()
	{
		state += solver_buffer_state_increment1;
	}

	void Integrator::DisplayParameters()
	{
		cout << "-------------" << endl;
		cout << "Number of Channel is :" << parameter.num_of_channels << ' \n ';
	}

	void Integrator::DisplayInitialCondition()
	{
		cout << "---------------------" << endl;
		cout << "Integration initial condition X_0:" << endl;
		cout << "The initial condition is:  " << endl;
		cout << state << endl;
	}

	Integrator::~Integrator()
	{
	}

	void TransferFunction::SystemMatricesFormStrictlyProperTF(const VectorXd& Numerator, 
															  const VectorXd& Denominator)
	{
		// use the controller  canonical form. reference : https://paginas.fe.up.pt/~mprocha/CL/English%20CL-5.pdf
		// the order is number of elements - 1
		int order_of_numerator = Numerator.size()-1;
		int order_of_denominator = Denominator.size()-1;

		VectorXd alpha;
		VectorXd beta;
		alpha.resize(order_of_denominator);
		beta.resize(order_of_denominator);
		alpha.setZero();
		beta.setZero();
		for (int i = 0; i < order_of_denominator; i++) {
			alpha(i) = Denominator(order_of_denominator - i) / Denominator(0);
		}

		for (int i = 0; i < order_of_numerator+1; i++) {
			beta(i) = Numerator(order_of_numerator-i)/ Denominator(0);
		}

		for (int i = 0; i < order_of_denominator; i++) {
			A(0, i) = - alpha(order_of_denominator - i -1);
		}
		A.block(1, 0, order_of_denominator-1, order_of_denominator-1) = MatrixXd::Identity(order_of_denominator-1, order_of_denominator-1);
		B(0, 0) = 1.0;
		for (int i = 0; i < order_of_denominator; i++) {
			C(0, i) = beta(order_of_denominator - i -1);
		}
	}

	TransferFunction::TransferFunction()
	{
	}

	TransferFunction::TransferFunction(const TransferFunctionParameter & parameter_)
	{
		system_info.category = LINEARSYSTEM;
		system_info.type = continous_TRANSFERFUNCTION;
		system_info.NO_CONTINUOUS_STATE = false;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		parameter = parameter_;
		// use minial realization to get the state space model
		int order_of_numerator = parameter_.Numerator.size() - 1;
		int order_of_denominator  = parameter_.Denominator.size() - 1;
		if (order_of_numerator > order_of_denominator) {
			// not proper 
			ready_to_run = false;
		}
		else {
			ready_to_run = true;
			// resize the matrices first
			A.resize(order_of_denominator, order_of_denominator);
			B.resize(order_of_denominator, 1);
			C.resize(1, order_of_denominator);
			D.resize(1, 1);
			// initialize them as zero matrices
			A.setZero();
			B.setZero();
			C.setZero();
			D.setZero();
			// if it is proper
			if (order_of_numerator < order_of_denominator) {
				// if the transfer function is strctly proper
				SystemMatricesFormStrictlyProperTF(parameter_.Numerator, parameter_.Denominator);
				system_info.DIRECT_FEED_THROUGH = false;
			}
			else {
				// if the order of the numerator is equal to the order of the denominator
				D(0, 0) = parameter_.Numerator(0) / parameter_.Denominator(0);
				system_info.DIRECT_FEED_THROUGH = true;
				VectorXd Num_Fac;//
				Num_Fac.resize(order_of_denominator);

				for (int i = 0; i < order_of_denominator; i++) {
					Num_Fac(i) = parameter_.Numerator(i+1) - D(0, 0) * parameter_.Denominator(i+1);
				}
				SystemMatricesFormStrictlyProperTF(Num_Fac, parameter_.Denominator);
			}
			state.resize(order_of_denominator);// assign the initial condition to the state
			state.setZero();
			// determine the size of the system
			system_info.num_of_continuous_states = order_of_denominator;
			system_info.num_of_inputs = 1;
			system_info.num_of_outputs = 1;
			// initialize state memeory
			state.resize(system_info.num_of_continuous_states);
			output.resize(system_info.num_of_outputs);
			system_info.input_connection.resize(system_info.num_of_inputs, 2);
		}
	}

	void TransferFunction::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// x_dot  = A x + B u
		derivative = A * state + B * input;
	}

	void TransferFunction::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		// y = Cx + Du
		output = C * state + D * input;
	}

	void TransferFunction::IncrementState()
	{
		state += solver_buffer_state_increment1;
	}

	void TransferFunction::DisplayParameters()
	{
		cout << "---------------------" << endl;
		cout << "The transfer function entered is: \n";

		int num_1 = parameter.Numerator.size();
		int dem_1 = parameter.Denominator.size();

		for (int i = 0; i < num_1 -1; i++) {
			cout << parameter.Numerator(i) << " s^(" << num_1 - 1 - i << ") + ";
		}
		cout << parameter.Numerator(num_1 - 1) << endl;
		cout << "---------------------" << endl;
		for (int j = 0; j < dem_1 - 1; j++) {
			cout << parameter.Denominator(j) << " s^(" << dem_1 - 1 - j << ") + ";
		}
		cout << parameter.Denominator(dem_1 - 1) << endl;
		if (ready_to_run) {
			cout << " The cooresponding state space model is: " << endl;
			cout << " A: " << endl;
			cout << A << endl;
			cout << " B: "<< endl;
			cout << B << endl;
			cout << " C: " << endl;
			cout << C << endl;
			cout << " D: " << endl;
			cout << D << endl;
		}
		else {
			cout << " Incorrect transfer function coefficients, check the numerator order \n ";
		}

	}

	void TransferFunction::DisplayInitialCondition()
	{
		cout << " Initial condition for the transfer function block is set to zero \n ";
	}

	TransferFunction::~TransferFunction()
	{
	}

}