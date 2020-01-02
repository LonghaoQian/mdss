#include "pch.h"
#include "Source_SignalGenerator.h"


Source_SignalGenerator::Source_SignalGenerator(const SignalGeneratorparameter& _parameter)
{
	system_info.DIRECT_FEED_THROUGH = false;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 0;
	system_info.system_type = Signal_Generator;
	system_info.num_of_outputs = _parameter.num_of_channels;
	parameter = _parameter;
	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);
	omega = _parameter.frequency * 2 * M_PI;
}

void Source_SignalGenerator::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
{
	
}

void Source_SignalGenerator::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	double temp = sin(omega*t + parameter.phase_shift);
	switch (parameter.type)
	{
	case SINE:
		for (int i = 0; i < system_info.num_of_outputs; i++)
		{
			output(i) = temp* parameter.amplitude;
		}
		break;
	case SQUARE:
		for (int i = 0; i < system_info.num_of_outputs; i++)
		{
			if (temp > 0)
			{
				output(i) = parameter.amplitude;
			}
			else
			{
				output(i) = -parameter.amplitude;
			}
		}
		break;
	case RAMP:

		break;
	default:
		for (int i = 0; i < system_info.num_of_outputs; i++)
		{
			output(i) = temp * parameter.amplitude;
		}
		break;
	}
}

void Source_SignalGenerator::IncrementState()
{
}

void Source_SignalGenerator::DisplayParameters()
{
}

void Source_SignalGenerator::DisplayInitialCondition()
{
}


Source_SignalGenerator::~Source_SignalGenerator()
{
}
