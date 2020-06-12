#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 02 15

Math blocks for simulation:

1. Constant block
2. Muliplication block
3. Gain block
4. Summation block
TO DO :
5. Saturation
_________________________________
*/

#include "Subsystem.h"
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

	struct SumParameter {
		int num_of_inputs;
		int input_dimensions;
		VectorXd sign_list;
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

	enum TrigonometryType{
		sin = 0,
		cos,
		tan,
		cot,
		asin,
		acos,
		atan,
		acot,
		atan2
	};

	struct TrigonometryParameter {
		int num_of_inputs;
		TrigonometryType type;
	};

	class TrigonometricFunction :
		public Subsystem
	{
	private:
			TrigonometryParameter para_;
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

	enum SpecialFunctionType {
		ln = 0,
		log10,
		exp,
		pow
	};

	class SpecialFunction :
		public Subsystem
	{
	private:
	public:
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
	};

	class Lookup1D :
		public Subsystem 
	{
	private:
	};

	class Lookup2D :
		public Subsystem
	{
	private:
	};

}
