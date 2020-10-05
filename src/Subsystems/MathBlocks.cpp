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
		system_info.category = MATH;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		param_ = _multiparameter;
		ready_to_run = false;
		system_info.system_parameter_ok = false;
		// M1*M2 
		// 1. (num_of_rows, num_of_cols) 
		// 2. (num_of_rows, num_of_cols)
		// input are assembled by the columns of both matrices 
		switch (param_.Mode)
		{
		case MULTI_MATRIX:
			// determine whether matrix dimension matches
			if (param_.input1_dimension(MATRIX_COL) == param_.input2_dimension(MATRIX_ROW)) { // col of M1 must equal to col of M2
				num_of_elements_input1 = param_.input1_dimension(MATRIX_ROW) * param_.input1_dimension(MATRIX_COL); // total number of inputs of 1
				num_of_elements_input2 = param_.input2_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL); // total number of inputs of 2
				system_info.num_of_inputs = num_of_elements_input1 + num_of_elements_input2;
				system_info.num_of_outputs = param_.input1_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL);
				ready_to_run = true;
				system_info.system_parameter_ok = true;
			}
			else {
				ready_to_run = false;
				system_info.system_parameter_ok = false;
			}
			break;
		case MULTI_ELEMENTWISE: // elementwise product of 2 matrices with the same size
			// determine 
			if ((param_.input1_dimension(MATRIX_ROW) == param_.input2_dimension(MATRIX_ROW)) && (param_.input1_dimension(MATRIX_COL) == param_.input2_dimension(MATRIX_COL))) {
				system_info.num_of_inputs = 2 * param_.input1_dimension(MATRIX_ROW) * param_.input1_dimension(MATRIX_COL);
				system_info.num_of_outputs = param_.input1_dimension(MATRIX_ROW) * param_.input1_dimension(MATRIX_COL);
				ready_to_run = true;
				system_info.system_parameter_ok = true;
			}
			else {
				ready_to_run = false;
				system_info.system_parameter_ok = false;
			}
			break;
		case MULTI_SCALAR: // multiply a vector with a scalar (as the 0th input)
			ready_to_run = true;
			system_info.system_parameter_ok = true;
			// set input 1 as the scalar
			param_.input1_dimension(MATRIX_ROW) = 1;
			param_.input1_dimension(MATRIX_COL) = 1;
			system_info.num_of_inputs = 1 + param_.input2_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL);
			system_info.num_of_outputs = param_.input2_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL);
			break;

		default: // default set to matrix
			if (param_.input1_dimension(MATRIX_COL) == param_.input2_dimension(MATRIX_ROW)) {
				system_info.num_of_inputs = param_.input1_dimension(MATRIX_ROW) * param_.input1_dimension(MATRIX_COL) + param_.input2_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL);
				system_info.num_of_outputs = param_.input1_dimension(MATRIX_ROW) * param_.input2_dimension(MATRIX_COL);
				ready_to_run = true;
				system_info.system_parameter_ok = true;
			}
			else {
				ready_to_run = false;
				system_info.system_parameter_ok = false;
			}
			break;
		}

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
		case MULTI_MATRIX:

			for (int i = 0; i < param_.input1_dimension(0); i++) {
				for (int j = 0; j < param_.input2_dimension(1); j++) {
					temp1 = 0.0; // clear temp1
					for (int k = 0; k < param_.input1_dimension(MATRIX_COL); k++) {
						temp1 += input(k*param_.input1_dimension(MATRIX_ROW) + i)*input(num_of_elements_input1 + j * param_.input2_dimension(MATRIX_ROW) + k);
					}
					output(i + param_.input1_dimension(MATRIX_ROW)*j) = temp1;
				}
			}

			break;
		case MULTI_ELEMENTWISE:
			// directly times the input elements
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = input(i) * input(i + system_info.num_of_outputs);
			}
			break;
		case MULTI_SCALAR:
			// input(0) is scalar
			output = input(0) * input.tail(system_info.num_of_outputs);
			break;
		}
	}

	void Multiplication::IncrementState()
	{
		// No increment state for multiplication block
	}

	void Multiplication::DisplayParameters()
	{
		switch (param_.Mode) {
		case MULTI_MATRIX:
			std::cout << "Mode is : Matrix " << std::endl;
			if (!ready_to_run) {
				std::cout << "The dimensions of input matrices are inconsistent! \n";
			}
			break;
		case MULTI_ELEMENTWISE:
			std::cout << "Mode is : ElementWise " << std::endl;
			if (!ready_to_run) {
				std::cout << "The dimensions of input matrices are inconsistent! \n";
			}
			break;
		case MULTI_SCALAR:
			std::cout << "Mode is : Scalar" << std::endl;
			break;
		}
		std::cout << "the dimension of the 1st input is set to: " << param_.input1_dimension(MATRIX_ROW) << " X " << param_.input1_dimension(MATRIX_COL) << std::endl;
		std::cout << "the dimension of the 2nd input is set to: " << param_.input2_dimension(MATRIX_ROW) << " X " << param_.input2_dimension(MATRIX_COL) << std::endl;
	}

	void Multiplication::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for product block----------" << std::endl;
	}


	Multiplication::~Multiplication()
	{

	}// cross-product block

	CrossProduct::CrossProduct(const CrossProductParameter & param)
	{
		system_info.type = math_CROSSPRODUCT;
		system_info.category = MATH;

		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;

		parameter = param;

		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs  = 6;
		system_info.num_of_outputs = 3;

		ready_to_run = true;
		system_info.system_parameter_ok = true;

		output.resize(system_info.num_of_outputs);
		output.setZero();

	}

	void CrossProduct::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// No increment state for cross-product block
	}

	void CrossProduct::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		output(mathauxiliary::VECTOR_X) = input(mathauxiliary::VECTOR_Y) * input(3 + mathauxiliary::VECTOR_Z) - input(mathauxiliary::VECTOR_Z) * input(3 + mathauxiliary::VECTOR_Y);
		output(mathauxiliary::VECTOR_Y) = input(mathauxiliary::VECTOR_Z) * input(3 + mathauxiliary::VECTOR_X) - input(mathauxiliary::VECTOR_X) * input(3 + mathauxiliary::VECTOR_Z);
		output(mathauxiliary::VECTOR_Z) = input(mathauxiliary::VECTOR_X) * input(3 + mathauxiliary::VECTOR_Y) - input(mathauxiliary::VECTOR_Y) * input(3 + mathauxiliary::VECTOR_X);
	}

	void CrossProduct::IncrementState()
	{
		// No increment state for cross product block
	}

	void CrossProduct::DisplayParameters()
	{
		std::cout << " No parameter for cross-product block \n";
	}

	void CrossProduct::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for cross-product block----------" << std::endl;
	}

	CrossProduct::~CrossProduct()
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
		system_info.category = MATH;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.system_parameter_ok = 0;
		ready_to_run = true;
		param_ = param;
		// calculate the number of total inputs 
		system_info.num_of_inputs = param_.input_dimensions * param_.SignList.size();
		num_of_channels = param_.SignList.size(); // number of channels
		Sign.resize(num_of_channels); 
		Sign.setZero();
		// set the output dimension
		system_info.num_of_outputs = param_.input_dimensions;
		output.resize(system_info.num_of_outputs);
		output.setZero();
		for (int i = 0; i < num_of_channels; i++) {
			if (param_.SignList[i] == SUM_POSITIVE) {// positive
				Sign(i) = 1.0;
			} else {// otherwise negative
				Sign(i) = -1.0;
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
		for (int i = 0; i < num_of_channels; i++) {
			output += Sign(i)*input.segment(i*param_.input_dimensions, param_.input_dimensions);
		}
	}

	void Sum::IncrementState()
	{
		// No increments for sum block
	}

	void Sum::DisplayParameters()
	{
		std::cout << "The input dimension is : " << param_.input_dimensions << '\n';
		std::cout << "The number of channels is : " << num_of_channels << '\n';
		std::cout << "The sign list is : \n";
		int j = 0;
		for (auto i : param_.SignList) {
			std::cout << "Sign # " << j << " : ";
			if (i == SUM_POSITIVE) {
				std::cout << " + ";
			}
			else {
				std::cout << " - ";
			}
			std::cout << '\n';
			j++;
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
		}
		else {
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

		if ((NumofDataReference_row == TableRows) && (NumofDataReference_col == TableCols)) {
			system_info.system_parameter_ok = 0;
			ready_to_run = true;

			std::cout << param_.reference_row << std::endl;
			std::cout << param_.reference_col << std::endl;

			table_.LoadTableData(param_.reference_row, param_.reference_col, param_.table, false);
		}
		else {
			system_info.system_parameter_ok = 1;
			ready_to_run = false;
		}

		system_info.num_of_inputs = 2;// number of total inputs 
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
			system_info.num_of_inputs = 2 * param_.num_of_channels;// number of total inputs for atan 2
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
				output(i) = atan2(input(i), input(param_.num_of_channels + i));// for atan 2 i and i + number of input channel are y and x inputs
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
		std::cout << "The function is selected as: " << TrigFunctionNameList[param_.type] << std::endl;
	}

	void TrigonometricFunction::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for trigonometric function block----------" << std::endl;
	}

	TrigonometricFunction::~TrigonometricFunction()
	{
	}

}