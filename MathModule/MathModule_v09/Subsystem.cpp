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

void Subsystem::UpdateSolverBuffer(unsigned int num_of_kn)
{
	solver_buffer_ouput_temp.resize(system_info.num_of_outputs);
	// k1,...,kn vector for numerical solver
	if (system_info.num_of_continuous_states == 0)
	{
		solver_buffer_k_sequence.resize(1, 1);
		system_info.NO_CONTINUOUS_STATE = true;
	}
	else {
		solver_buffer_k_sequence.resize(system_info.num_of_continuous_states, num_of_kn);
		solver_buffer_k_sequence.setZero(system_info.num_of_continuous_states, num_of_kn);
		system_info.NO_CONTINUOUS_STATE = false;
	}
	// set iniital value 
	solver_buffer_ouput_temp.setZero(system_info.num_of_outputs);
}

subsystem_info Subsystem::GetSystemInfo()
{
	return system_info;
}


