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
	// define the external function pointers
	typedef std::function<void(const double& t, const VectorXd&, const VectorXd&, VectorXd&)> DerivativeFunctionPtr;
	typedef std::function<void(const double& t, const VectorXd&, const VectorXd&, VectorXd&)> OutputFunctionPtr;

	struct ExternalFunctionParameter {
		OutputFunctionPtr output_ptr{ NULL };
		unsigned int  num_of_inputs{1};
		unsigned int  num_of_outputs{1};
	};

	class ExternalFunction :
		public Subsystem
	{
	public:
		ExternalFunction(const ExternalFunctionParameter& param);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		~ExternalFunction();
	private:
		ExternalFunctionParameter parameter;
	};

	struct ExternalSystemParameter {
		DerivativeFunctionPtr derivative_ptr{NULL};
		OutputFunctionPtr output_ptr{NULL};
		unsigned int  num_of_inputs{1};
		unsigned int  num_of_outputs{1};
		unsigned int  num_of_states{1};
		bool IsDirectFeedThrough{ false };
		bool NoContiunousState{ false };
	};

	struct ExternalSystemInitialCondition {
		VectorXd X0;
	};

	class ExternalSystem :
		public Subsystem
	{
	public:
		ExternalSystem(const ExternalSystemParameter param, const ExternalSystemInitialCondition& initial_condition);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		~ExternalSystem();
	private:
		ExternalSystemParameter parameter;
		VectorXd InitialCondition;
	};
}