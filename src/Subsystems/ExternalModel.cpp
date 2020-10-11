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
		std::cout << "----------------- \n";
		std::cout << "The number of input is: " << system_info.num_of_inputs << ", the number of output is: " << system_info.num_of_outputs << "\n";
	}

	void ExternalFunction::DisplayInitialCondition()
	{
		std::cout << "------- No initial condition for external function block -------" << std::endl;
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
		system_info.category = EXTERNAL;
		system_info.type = external_FUNCTION;
		system_info.DIRECT_FEED_THROUGH = param.IsDirectFeedThrough;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.num_of_continuous_states = param.num_of_states;
		system_info.num_of_inputs = param.num_of_inputs;
		system_info.num_of_outputs = param.num_of_outputs;
		system_info.NO_CONTINUOUS_STATE = param.NoContiunousState;

		if (!system_info.NO_CONTINUOUS_STATE) {
			// loading initial condition:
			InitialCondition = initial_condition.X0;
			state.resize(system_info.num_of_continuous_states);
			state = initial_condition.X0;
		}

		parameter = param;
	}

	void ExternalSystem::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		if (!system_info.NO_CONTINUOUS_STATE) {
			parameter.derivative_ptr(t, state, input, derivative);
		}
	}

	void ExternalSystem::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		parameter.output_ptr(t, state, input, output);
	}

	void ExternalSystem::DisplayParameters()
	{
		std::cout << "----------------- \n";
		
		std::cout << "The number of input is: " << system_info.num_of_inputs << ", the number of output is: " << system_info.num_of_outputs << "\n";
		if(system_info.NO_CONTINUOUS_STATE){
			std::cout << "No contnuous state! \n";
		}
		else {
			std::cout << "The number of state is: " << system_info.num_of_continuous_states << "\n";
		}
	}

	void ExternalSystem::DisplayInitialCondition()
	{
		std::cout << "----------------- \n";
		std::cout << "The initial condition is: \n";
		std::cout << InitialCondition;
	}

	void ExternalSystem::IncrementState()
	{
		if (!system_info.NO_CONTINUOUS_STATE) {
			state += solver_buffer_state_increment1;
		}
	}

	ExternalSystem::~ExternalSystem()
	{
	}

}