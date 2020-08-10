#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 07 17

Math blocks contains the following blocks:

1. Constant block
2. Muliplication block
3. Gain block
4. Summation block
5. Special Functions Block
6. Trigonometry Functions Block
7. 1D Lookup Table Block
8. 2D Lookup Table Block
_________________________________
*/

#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace mathblocks {

	struct ConstantParameter {
		VectorXd value;
	};

	class Constant :
		public Subsystem
	{
	private:
		ConstantParameter param_;
	public:
		Constant(const ConstantParameter& param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Constant();
	};
	// Define input mode
	enum InputMode { ElementWise, Matrix, Scalar };

	struct MultiplicationParam {
		InputMode Mode;
		Vector2i input1_dimension;
		Vector2i input2_dimension;
	};

	class Multiplication :
		public Subsystem
	{
	private:
		MultiplicationParam param_;
		Eigen::MatrixXd M_1;
		Eigen::MatrixXd M_2;
		Eigen::MatrixXd M;
		int num_of_elements_input1;
		int num_of_elements_input2;
	public:
		Multiplication(const  MultiplicationParam& _multiparameter);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Multiplication();

	};
	// Gain Block
	//-----------------------------------------------//
	struct GainParameter {
		MatrixXd K;
		InputMode Mode;
		int num_of_inputs;
	};

	class Gain :
		public Subsystem
	{
	private:
		InputMode Mode;
		MatrixXd K;
	public:
		Gain(const GainParameter& _gainparameter);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		/*-----Only output function is used, the rest is set to empty functions just to keep the subsystem format-------------------*/
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& temp_derivative);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Gain();
	};
	//-----------------------------------------------//
	struct SumParameter {
		int num_of_inputs;
		int input_dimensions;
		VectorXd sign_list;
	};

	enum SumSignList {
		SUM_POSITIVE = 1,
		SUM_NEGATIVE = -1
	};

	class Sum :
		public Subsystem
	{
	private:
		SumParameter param_;
	public:
		Sum(const SumParameter& param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Sum();
	};
	//-----------------------------------------------//
	enum TrigonometryType{
		SIN = 0,
		COS,
		TAN,
		COT,
		ASIN,
		ACOS,
		ATAN,
		ACOT,
		ATAN2
	};

	// function dispatch table for trigonometry functions (locally global variable)
	// C++ math function table http://www.cplusplus.com/reference/cmath/
	static std::map< const TrigonometryType, std::function<double(double)> > TrigFunctionSingleInput{
	{TrigonometryType::SIN,[](double input) { return sin(input); } },
	{TrigonometryType::COS,[](double input) { return cos(input); } },
	{TrigonometryType::TAN,[](double input) { return tan(input); } },
	{TrigonometryType::COT,[](double input) { return 1/tan(input); } },
	{TrigonometryType::ASIN,[](double input) { return asin(input); } },
	{TrigonometryType::ACOS,[](double input) { return acos(input); } },
	{TrigonometryType::ATAN,[](double input) { return atan(input); } },
	{TrigonometryType::ACOT,[](double input) { return M_PI/2 - atan(input); } }, // source: https://colalg.math.csusb.edu/~devel/IT/main/m06_inverse/src/s02_tanflip.html
	};

	static std::map<const TrigonometryType, std::string> TrigFunctionNameList{
		{TrigonometryType::SIN,"sin"},
		{TrigonometryType::COS,"cos"},
		{TrigonometryType::TAN,"tan"},
		{TrigonometryType::COT,"cot"},
		{TrigonometryType::ASIN,"arcsin"},
		{TrigonometryType::ACOS,"arccos"},
		{TrigonometryType::ATAN,"arctan"},
		{TrigonometryType::ACOT,"arccot"},
		{TrigonometryType::ATAN2,"arctan2"},
	};

	struct TrigonometryParameter {
		int num_of_channels;
		TrigonometryType type;
	};

	class TrigonometricFunction :
		public Subsystem
	{
	private:
			TrigonometryParameter param_;
			std::map<const TrigonometryType, std::function<double(double)>>::iterator TargetFunction;
	public:
		TrigonometricFunction();
		TrigonometricFunction(const TrigonometryParameter& param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~TrigonometricFunction();
	};
	//-----------------------------------------------//
	enum SpecialFunctionType {
		LN = 0,
		LOG10,
		EXP,
		POW,
		SQRT
	};

	struct SpecialFunctionParameter {
		int num_of_channels;
		SpecialFunctionType type;
	};

	class SpecialFunction :
		public Subsystem
	{
	private:
		SpecialFunctionParameter param_;
	public:
		SpecialFunction(const SpecialFunctionParameter param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~SpecialFunction();
	};

	struct Lookup1DParameter {
		VectorXd reference;
		VectorXd table;
	};
	//-----------------------------------------------//
	class Lookup1D :
		public Subsystem 
	{
	private:
		Lookup1DParameter param_;
		mathauxiliary::Lookup_1D table_;
	public:
		Lookup1D(const Lookup1DParameter param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Lookup1D();
	};
	//-----------------------------------------------//
	// 2-D lookup table input(0) = row input(1) = col
	// https://itectec.com/matlab/matlab-how-exactly-does-simulink-perform-2-d-interpolation-when-using-the-2-d-lookup-table-block/
	struct Lookup2DParameter {
		VectorXd reference_row;
		VectorXd reference_col;
		MatrixXd table;
	};

	enum {
		LOOKUP_INPUT_ROW = 0,
		LOOKUP_INPUT_COL,
	};

	class Lookup2D :
		public Subsystem
	{
	private:
		Lookup2DParameter param_;
		mathauxiliary::Lookup_2D table_;
	public:
		Lookup2D(const Lookup2DParameter param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Lookup2D();
	};

}
