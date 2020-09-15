#include "pch.h"
#include "ExternalModel.h"
namespace externalmodel{
	/*--------------- external function ---------------*/
	ExternalFunction::ExternalFunction()
	{
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
	ExternalSystem::ExternalSystem()
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