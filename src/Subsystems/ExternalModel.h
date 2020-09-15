#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 09 13

external libs contains the following blocks:

1. external model
2. external function

_________________________________
*/
#include "Subsystem.h"

namespace externalmodel {
	typedef std::function<void(const double& t, const VectorXd&, const VectorXd&, VectorXd&)> DerivativeFunctionPtr;
	typedef std::function<void(const double& t, const VectorXd&, const VectorXd&, VectorXd&)> OutputFunctionPtr;

	struct ExternalFunctionParameter {
		OutputFunctionPtr output_ptr;
		unsigned int  num_of_inputs;
		unsigned int  num_of_outputs;
	};

	class ExternalFunction :
		public Subsystem
	{
	private:
		ExternalFunctionParameter parameter;
	public:
		ExternalFunction();
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		~ExternalFunction();
	};

	struct ExternalSystemParameter {
		DerivativeFunctionPtr derivative_ptr;
		OutputFunctionPtr output_ptr;
		unsigned int  num_of_inputs;
		unsigned int  num_of_outputs;
		unsigned int  num_of_states;
	};

	class ExternalSystem :
		public Subsystem
	{
	private:
		ExternalSystemParameter parameter;
	public:
		ExternalSystem();
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		~ExternalSystem();
	};
}