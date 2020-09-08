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
	for (int i = 0; i < param_.num_of_inputs; i++) {
		if (param_.sign_list(i) > 0) {
			param_.sign_list(i) = 1.0;
		}
		else {
			param_.sign_list(i) = -1.0;
		}
	}

}

void Sum::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equations for sum block
}

void Sum::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	// reset the output to zero
	output.setZero();
	for (int i = 0; i < param_.num_of_inputs; i++) {
		output += param_.sign_list(i)*input.segment(i*param_.input_dimensions, param_.input_dimensions);
	}
}

void Sum::IncrementState()
{
	// No increments for sum block
}

void Sum::DisplayParameters()
{
	std::cout << "Input dimension is : " << param_.input_dimensions << '\n';
	std::cout << "Number of inputs is : " << param_.num_of_inputs << '\n';
	std::cout << "The sign list is : " << '\n';
	for (int i = 0; i < param_.num_of_inputs; i++) {
		std::cout << " Input # " << i << " is ";
		if (param_.sign_list(i) > 0) {
			std::cout << " + \n";
		}
		else {
			std::cout << " - \n";
		}
	}
}

void Sum::DisplayInitialCondition()
{
	std::cout << "------No initial condition for sum block----------" << std::endl;
}

Sum::~Sum()
{
}

Lookup1D::Lookup1D(const Lookup1DParameter param)
{
	system_info.type = math_LOOKUP1D;
	system_info.category = MATH;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;

	param_ = param;

	auto NumofDataPoints = param_.reference.rows(); // number of data points
	auto NumofDataChannels = param_.table.cols(); // number of output channels
	auto NumofTableDataPoints = param_.table.rows();// number of data points of table

	if (NumofDataPoints == NumofTableDataPoints) {
		system_info.system_parameter_ok = 0;
		ready_to_run = true;
		table_.LoadTableData(param_.reference, param_.table, false);
	}
	else {
		system_info.system_parameter_ok = 1;
		ready_to_run = false;
	}

	system_info.num_of_inputs = 1;// number of total inputs 
	system_info.num_of_outputs = NumofDataChannels;
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void Lookup1D::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equations for lookup block
}

void Lookup1D::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	table_.GetOutput(output, input(0));
}

void Lookup1D::IncrementState()
{
	// No IncrementState for lookup block
}

void Lookup1D::DisplayParameters()
{
	if (system_info.system_parameter_ok == 0) {
		std::cout << "The lookup reference data is:  " << std::endl;
		std::cout << param_.reference << std::endl;
		std::cout << "The table data is:  " << std::endl;
		std::cout << param_.table << std::endl;
	}else{
		std::cout << "Incorrect Table File!  " << std::endl;
	}
}

void Lookup1D::DisplayInitialCondition()
{
	std::cout << "------No initial condition for lookup block----------" << std::endl;
}

Lookup1D::~Lookup1D()
{
}

Lookup2D::Lookup2D(const Lookup2DParameter param)
{
	system_info.type = math_LOOKUP2D;
	system_info.category = MATH;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;

	param_ = param;

	auto NumofDataReference_row = param_.reference_row.rows(); // number of data points for row reference
	auto NumofDataReference_col = param_.reference_col.rows(); // number of data points for column reference

	auto TableRows = param_.table.rows();// number of row data points of table
	auto TableCols = param_.table.cols();// number of column data points of table

	if ((NumofDataReference_row == TableRows) && (NumofDataReference_col == TableCols) ) {
		system_info.system_parameter_ok = 0;
		ready_to_run = true;

		std::cout << param_.reference_row << std::endl;
		std::cout << param_.reference_col << std::endl;

		table_.LoadTableData(param_.reference_row, param_.reference_col,param_.table, false);
	}
	else {
		system_info.system_parameter_ok = 1;
		ready_to_run = false;
	}

	system_info.num_of_inputs  = 2;// number of total inputs 
	system_info.num_of_outputs = 1;
	output.resize(system_info.num_of_outputs);
	output.setZero();
}

void Lookup2D::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// No differential equations for lookup block
}

void Lookup2D::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	output(0) = table_.GetOutput(input(LOOKUP_INPUT_ROW), input(LOOKUP_INPUT_COL));
}

void Lookup2D::IncrementState()
{
	// No IncrementState for lookup block
}

void Lookup2D::DisplayParameters()
{
	if (system_info.system_parameter_ok == 0) {
		std::cout << "The lookup row reference data is:  " << std::endl;
		std::cout << param_.reference_row << std::endl;
		std::cout << "The lookup colume reference data is:  " << std::endl;
		std::cout << param_.reference_col << std::endl;
		std::cout << "The table data is:  " << std::endl;
		std::cout << param_.table << std::endl;
	}
	else {
		std::cout << "Incorrect Table File!  " << std::endl;
	}
}

void Lookup2D::DisplayInitialCondition()
{
	std::cout << "------No initial condition for lookup block----------" << std::endl;
}

Lookup2D::~Lookup2D()
{
}
// trig -------------------------------------------------------------
TrigonometricFunction::TrigonometricFunction()
{
}

TrigonometricFunction::TrigonometricFunction(const TrigonometryParameter & param)
{
	system_info.type = math_TRIGONOMETRYFUNCTION;
	system_info.category = MATH;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	param_ = param;

	if (param_.type == TrigonometryType::ATAN2) {
		system_info.num_of_inputs = 2* param_.num_of_channels;// number of total inputs for atan 2
	}
	else
	{
		system_info.num_of_inputs = param_.num_of_channels;// number of total inputs 
		TargetFunction = TrigFunctionSingleInput.find(param_.type);// determine which function is used before running
		if (TargetFunction != TrigFunctionSingleInput.end()) {
			system_info.system_parameter_ok = 0;
			ready_to_run = true;
		}
		else {
			std::cout << "Error Function Type\n";
			system_info.system_parameter_ok = 1;
			ready_to_run = false;
		}
	}

	system_info.num_of_outputs = param_.num_of_channels;// same as number of 
	output.resize(system_info.num_of_outputs);
	output.setZero();

}

void TrigonometricFunction::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// no differential equation for trigonometric function block
}

void TrigonometricFunction::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	if (param_.type == TrigonometryType::ATAN2) {
		for (int i = 0; i < param_.num_of_channels; i++) {
			output(i) = atan2(input(i),input(param_.num_of_channels+i));// for atan 2 i and i + number of input channel are y and x inputs
		}
	}
	else
	{
		for (int i = 0; i < param_.num_of_channels; i++) {
			output(i) = TargetFunction->second(input(i));
		}
	}

}

void TrigonometricFunction::IncrementState()
{
	// no increment state for trigonometric function block
}

void TrigonometricFunction::DisplayParameters()
{
	std::cout << "The function is selected as: " << TrigFunctionNameList [param_.type]<< std::endl;
}

void TrigonometricFunction::DisplayInitialCondition()
{
	std::cout << "------No initial condition for trigonometric function block----------" << std::endl;
}

TrigonometricFunction::~TrigonometricFunction()
{
}

}