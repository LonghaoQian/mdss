#include "pch.h"
#include "ExternalModel.h"
namespace externalmodel{
	/*--------------- external function ---------------*/
	ExternalFunction::ExternalFunction(const ExternalFunctionParameter& param)
	{
		system_info.category = EXTERNAL;
		system_info.type = external_FUNCTION;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = param.num_of_inputs;
		system_info.num_of_outputs = param.num_of_outputs;
		parameter = param;
	}

	void ExternalFunction::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no differential equation for external function
	}

	void ExternalFunction::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		parameter.output_ptr(t, state, input, output);
	}

	void ExternalFunction::DisplayParameters()
	{

	}

	void ExternalFunction::DisplayInitialCondition()
	{

	}

	void ExternalFunction::IncrementState()
	{
		// no incremental state
	}
	
	ExternalFunction::~ExternalFunction()
	{
	}
	/*-------------------- external system ----------------------------*/
	ExternalSystem::ExternalSystem(const ExternalSystemParameter param, const ExternalSystemInitialCondition& initial_condition)
	{
	}

	void ExternalSystem::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
	}

	void ExternalSystem::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
	}

	void ExternalSystem::DisplayParameters()
	{
	}

	void ExternalSystem::DisplayInitialCondition()
	{
	}

	void ExternalSystem::IncrementState()
	{
	}

	ExternalSystem::~ExternalSystem()
	{
	}

}