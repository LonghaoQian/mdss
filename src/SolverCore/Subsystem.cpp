#include "pch.h"
#include "Subsystem.h"




Subsystem::Subsystem()
{
}


Subsystem::~Subsystem()
{
}

VectorXd Subsystem::GetState()
{
	return state;
}

void Subsystem::UpdateOutput(const double& t, const double& current_stepsize)
{
	OutputEquation(t+ current_stepsize, state, solver_buffer_input_temp, output);
}

VectorXd Subsystem::GetOutput()
{
	return output;
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
	// initialize the increment first
	solver_buffer_state_increment1.setZero(system_info.num_of_continuous_states);
	solver_buffer_state_increment2.setZero(system_info.num_of_continuous_states);
	relative_error.resize(system_info.num_of_continuous_states);// initialize relative error
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

VectorXd Subsystem::Solver_GetOuputTemp()
{
	return solver_buffer_output_temp;
}

void Subsystem::Solver_CalculateIncrement(const VectorXd & updatecoefficients)
{
	if (!system_info.NO_CONTINUOUS_STATE)
	{
		//solver_buffer_state_increment1.setZero(system_info.num_of_continuous_states);// set to zero before added ? why specifying dimension? 
		solver_buffer_state_increment1.setZero(); // setzero without resize
		for (int i = 0; i < updatecoefficients.size(); i++)
		{
			solver_buffer_state_increment1 += updatecoefficients(i)*solver_buffer_k_sequence.col(i);
		}
	}
}

double Subsystem::Solver_CalculateIncrement(const VectorXd & updatecoefficients1, const VectorXd & updatecoefficients2)
{
	if (!system_info.NO_CONTINUOUS_STATE)
	{

		//solver_buffer_state_increment1.setZero(system_info.num_of_continuous_states);// set to zero before added
		solver_buffer_state_increment1.setZero();
		for (int i = 0; i < updatecoefficients1.size(); i++)
		{
			solver_buffer_state_increment1 += updatecoefficients1(i)*solver_buffer_k_sequence.col(i);
		}
		//solver_buffer_state_increment2.setZero(system_info.num_of_continuous_states);// set to zero before added
		solver_buffer_state_increment2.setZero();
		for (int i = 0; i < updatecoefficients2.size(); i++)
		{
			solver_buffer_state_increment2 += updatecoefficients2(i)*solver_buffer_k_sequence.col(i);
		}
		// calculate relative error
		//VectorXd relative_error;
		// relative_error.resize(system_info.num_of_continuous_states);
		for (unsigned int i = 0; i < system_info.num_of_continuous_states; i++)
		{
			if (abs(state(i)) > 1)// if this state is greater than 1, then normalize it to 1;
			{
				relative_error(i) = (solver_buffer_state_increment1(i) - solver_buffer_state_increment2(i)) / abs(state(i));
			}
			else {
				relative_error(i) = solver_buffer_state_increment1(i) - solver_buffer_state_increment2(i);
			}
		}
		return relative_error.norm();
	}
	else {
		return 0;
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


