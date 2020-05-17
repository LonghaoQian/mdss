#include "pch.h"
#include "MathBlocks.h"

namespace mathblocks {

// Constant Block 
Constant::Constant(const ConstantParameter& param)
{
	system_info.type = math_CONSTANT;
	system_info.DIRECT_FEED_THROUGH = false;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;

	param_ = param;

	system_info.num_of_inputs = 0;
	system_info.num_of_outputs = param_.value.size();

	output.resize(system_info.num_of_outputs);
	output = param_.value;// directly set the output as the constant value
	ready_to_run = true;

}

void Constant::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equation for constant block
}

void Constant::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	output = param_.value;
}

void Constant::IncrementState()
{
	// No increment State for constant block
}

void Constant::DisplayParameters()
{
	std::cout << "Constant Value is :" << std::endl;
	std::cout << "V = " << std::endl << param_.value << std::endl;
}

void Constant::DisplayInitialCondition()
{
	std::cout << "------No initial condition for constant block----------" << std::endl;
}


Constant::~Constant()
{

}


// Multiplication Block

Multiplication::Multiplication(const  MultiplicationParam& _multiparameter)
{
	system_info.type = math_PRODUCT;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	param_ = _multiparameter;
	ready_to_run = false;
	system_info.system_parameter_ok = false;
	switch (param_.Mode)
	{
	case Matrix:
		// determine whether matrix dimension matches
		if (param_.input1_dimension(1) == param_.input2_dimension(0)) {
			num_of_elements_input1 = param_.input1_dimension(0) * param_.input1_dimension(1);
			num_of_elements_input2 = param_.input2_dimension(0) * param_.input2_dimension(1);
			system_info.num_of_inputs = num_of_elements_input1 + num_of_elements_input2;
			system_info.num_of_outputs = param_.input1_dimension(0) * param_.input2_dimension(1);
			ready_to_run = true;
			system_info.system_parameter_ok = true;
		}
		break;
	case ElementWise:
		// determine 
		if ((param_.input1_dimension(0) == param_.input2_dimension(0)) && (param_.input1_dimension(1) == param_.input2_dimension(1))) {
			system_info.num_of_inputs = 2 * param_.input1_dimension(0) * param_.input1_dimension(1);
			system_info.num_of_outputs = param_.input1_dimension(0) * param_.input1_dimension(1);
			ready_to_run = true;
			system_info.system_parameter_ok = true;
		}
		break;
	case Scalar:
		ready_to_run = true;
		system_info.system_parameter_ok = true;
		// set input 1 as the scalar
		param_.input1_dimension(0) = 1;
		param_.input1_dimension(1) = 1;
		system_info.num_of_inputs =  1 + param_.input2_dimension(0) * param_.input2_dimension(1);
		system_info.num_of_outputs = param_.input2_dimension(0) * param_.input2_dimension(1);
		break;

	default: // default set to matrix
				// determine whether matrix dimension matches
		if (param_.input1_dimension(1) == param_.input2_dimension(0)) {
			system_info.num_of_inputs = param_.input1_dimension(0) * param_.input1_dimension(1) + param_.input2_dimension(0) * param_.input2_dimension(1);
			system_info.num_of_outputs = param_.input1_dimension(0) * param_.input2_dimension(1);
			ready_to_run = true;
			system_info.system_parameter_ok = true;
		}
		break;
	}

	M_1.resize(param_.input1_dimension(0), param_.input1_dimension(1));
	M_2.resize(param_.input2_dimension(0), param_.input2_dimension(1));
	M.resize(param_.input1_dimension(0), param_.input2_dimension(1));
	M_1.setZero();
	M_2.setZero();
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void Multiplication::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equation for multiplication block
}

void Multiplication::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	// reconstruct matrix using inputs
	switch (param_.Mode)
	{
	case Matrix:
		for (int i = 0; i < param_.input1_dimension(1); i++) {
			M_1.col(i) = input.segment(i*param_.input1_dimension(0), param_.input1_dimension(0));
		}
		for (int i = 0; i < param_.input2_dimension(1); i++) {
			M_2.col(i) = input.segment(num_of_elements_input1 + i * param_.input2_dimension(0), param_.input2_dimension(0));
		}
		M = M_1 * M_2;
		for (int i = 0; i < param_.input2_dimension(1); i ++) {
			output.segment(i*param_.input1_dimension(0), param_.input1_dimension(0));
		}
		break;
	case ElementWise:
		// directly times the input elements
		for (int i = 0; i < system_info.num_of_outputs; i++) {
			output(i) = input(i) * input(i + system_info.num_of_outputs);
		}
		break;
	case Scalar:
		// input(0) is scalar
		output = input(0) * input.tail(system_info.num_of_outputs);
		break;
	default:
		break;
	}
	//output = 
}

void Multiplication::IncrementState()
{
	// No increment state for multiplication block
}

void Multiplication::DisplayParameters()
{
	switch (param_.Mode) {
	case Matrix:
		std::cout << "Mode is : Matrix " << std::endl;
		break;
	case ElementWise:
		std::cout << "Mode is : ElementWise " << std::endl;
		break;
	case  Scalar:
		std::cout << "Mode is : Scalar" << std::endl;
		break;
	default:
		break;
	}
	std::cout << "the dimension of the 1st input is set to: "<< param_.input1_dimension(0) << " X " << param_.input1_dimension(1) << std::endl;
	std::cout << "the dimension of the 2nd input is set to: " << param_.input2_dimension(0) << " X " << param_.input2_dimension(1) << std::endl;
}

void Multiplication::DisplayInitialCondition()
{
	std::cout << "------No initial condition for multiplication block----------" << std::endl;
}


Multiplication::~Multiplication()
{

}

// Gain Block

Gain::Gain(const GainParameter& _gainparameter)
{
	system_info.type = math_GAIN;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	K = _gainparameter.K;
	system_info.system_parameter_ok = 0;
	ready_to_run = true;
	Mode = _gainparameter.Mode;
	switch (_gainparameter.Mode)
	{
	case Matrix:
		system_info.num_of_inputs = _gainparameter.K.cols();
		system_info.num_of_outputs = _gainparameter.K.rows();
		break;
	case ElementWise:
		system_info.num_of_inputs = _gainparameter.K.rows();
		system_info.num_of_outputs = system_info.num_of_inputs;
		break;
	case Scalar:
		system_info.num_of_inputs = _gainparameter.num_of_inputs;
		system_info.num_of_outputs = _gainparameter.num_of_inputs;
		break;
	default:
		system_info.num_of_inputs = _gainparameter.K.cols();
		system_info.num_of_outputs = _gainparameter.K.rows();
		break;
	}
	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);
	ready_to_run = true;
}

void Gain::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{

	switch (Mode)
	{
	case Matrix:
		output = K * input;
		break;
	case ElementWise:
		output.resize(system_info.num_of_outputs);
		for (int i = 0; i < system_info.num_of_outputs; i++)
		{
			output(i) = input(i)*K(i, 0);
		}
		break;
	case Scalar:
		output = K(0, 0) * input;
		break;
	default:
		output = K * input;
		break;
	}
}

void Gain::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
{
	// No differential equation for gain block
}

void Gain::IncrementState()
{
	// No increment state for gain block
}

void Gain::DisplayParameters()
{
	std::cout << "Gain is :" << std::endl;
	std::cout << "K = " << std::endl << K << std::endl;
	switch (Mode) {
	case Matrix:
		std::cout << "Mode is : Matrix " << std::endl;
		break;
	case ElementWise:
		std::cout << "Mode is : ElementWise " << std::endl;
		break;
	case  Scalar:
		std::cout << "Mode is : Scalar" << std::endl;
		break;
	default:
		break;
	}
}
void Gain::DisplayInitialCondition()
{
	std::cout << "------No initial condition for gain block----------" << std::endl;
}

Gain::~Gain()
{
}

// Summation Block



Sum::Sum(const SumParameter & param)
{
	system_info.type = math_SUM;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.system_parameter_ok = 0;
	ready_to_run = true;
	param_ = param;

	system_info.num_of_inputs = param_.input_dimensions * param_.num_of_inputs;// number of total inputs 
	system_info.num_of_outputs = param_.input_dimensions;
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void Sum::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equations for sum block
}

void Sum::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	// 
	for (int i = 0; i < param_.num_of_inputs; i++) {
		output += input.segment(i*param_.input_dimensions, param_.input_dimensions);
	}
}

void Sum::IncrementState()
{
	// No increments for sum block
}

void Sum::DisplayParameters()
{
	std::cout << "Input dimension is : " << param_.input_dimensions << std::endl;
	std::cout << "Number of inputs is : " << param_.num_of_inputs << std::endl;
}

void Sum::DisplayInitialCondition()
{
	std::cout << "------No initial condition for sum block----------" << std::endl;
}

Sum::~Sum()
{
}

}