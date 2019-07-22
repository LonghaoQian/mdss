#include "stdafx.h"
#include "Subsystem.h"




Subsystem::Subsystem()
{
}


Subsystem::~Subsystem()
{
}

void Subsystem::SetInputConnection(const MatrixX2i& connection)
{
	system_info.input_connection = connection;
}

void Subsystem::OverrideDirectFeedThroughFlag(bool isDirectFeedThrough)
{
	system_info.DIRECT_FEED_THROUGH = isDirectFeedThrough;
}

void Subsystem::Solver_InitSolverBuffer(unsigned int num_of_kn)
{
	solver_buffer_output_temp.resize(system_info.num_of_outputs);
	solver_buffer_input_temp.resize(system_info.num_of_inputs);
	// k1,...,kn vector for numerical solver
	if (system_info.num_of_continuous_states == 0)
	{
		solver_buffer_k_sequence.resize(1, 1);
		system_info.NO_CONTINUOUS_STATE = true;
	}
	else {
		solver_buffer_k_sequence.resize(system_info.num_of_continuous_states, num_of_kn);
		solver_buffer_k_sequence.setZero(system_info.num_of_continuous_states, num_of_kn);

		solver_buffer_state_temp.resize(system_info.num_of_continuous_states);
		solver_buffer_Ki_temp.resize(system_info.num_of_continuous_states);

		system_info.NO_CONTINUOUS_STATE = false;
	}
}

void Subsystem::Solver_UpdateKiBuffer(int index, double& current_time,  double& stepsize, const MatrixXd& butchertableau)
{
	
	if (!system_info.NO_CONTINUOUS_STATE)
	{
		DifferentialEquation(current_time+ butchertableau(index, 0)*stepsize, solver_buffer_state_temp, solver_buffer_input_temp, solver_buffer_Ki_temp);
		solver_buffer_k_sequence.block(0, index, system_info.num_of_continuous_states, 1) = stepsize * solver_buffer_Ki_temp;
	}

}

void Subsystem::Solver_UpdateInputTemp(int index, double input_temp_i)
{
	solver_buffer_input_temp(index) = input_temp_i;
}


void Subsystem::Solver_PreturbState(int index,  const MatrixXd & butchertableau)
{
	if (!system_info.NO_CONTINUOUS_STATE)
	{
		solver_buffer_state_temp = state;
		for (int i = 0; i < index; i++)// calculate state pertubation and store the pertubed state in solver_buffer_state_temp
		{
			solver_buffer_state_temp += butchertableau(index, i + 1)*solver_buffer_k_sequence.block(0, i, system_info.num_of_continuous_states, 1);
		}
	}
}

VectorXd Subsystem::Solver_GetInputTemp()
{
	return solver_buffer_input_temp;
}

void Subsystem::Solver_PreturbOutput(int index, double& current_time, double& stepsize, const MatrixXd& butchertableau)
{
	// perturbed_t = t + t_pertubation t_pertubation is calculated in the simcontrol class
	OutputEquation(current_time + butchertableau(index, 0)*stepsize, solver_buffer_state_temp, solver_buffer_input_temp, solver_buffer_output_temp);
}

subsystem_info Subsystem::GetSystemInfo()
{
	return system_info;
}


