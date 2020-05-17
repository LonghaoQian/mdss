#include "pch.h"
#include "Source_SignalGenerator.h"

namespace source_sink {
	Source_SignalGenerator::Source_SignalGenerator(const SignalGeneratorparameter& _parameter)
	{
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 0;
		system_info.type = source_SINGALGENERATOR;
		system_info.num_of_outputs = _parameter.num_of_channels;
		parameter = _parameter;
		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);
		omega = _parameter.frequency * 2 * M_PI;// calculate angular velocity
		temp = 0.0;
	}

	void Source_SignalGenerator::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
	{

	}

	void Source_SignalGenerator::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		temp = sin(omega*t + parameter.phase_shift);
		switch (parameter.waveshape)
		{
		case SINE:
			for (int i = 0; i < system_info.num_of_outputs; i++)
			{
				output(i) = temp * parameter.amplitude;
			}
			break;
		case SQUARE:
			for (int i = 0; i < system_info.num_of_outputs; i++){
				if (temp > 0){
					output(i) = parameter.amplitude;
				}
				else{
					output(i) = -parameter.amplitude;
				}
			}
			break;
		case TRANGLE:
			// TO DO: use sine for the time being 
			for (int i = 0; i < system_info.num_of_outputs; i++)
			{
				output(i) = temp * parameter.amplitude;
			}
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
		// No increment state for signal generator
	}

	void Source_SignalGenerator::DisplayParameters()
	{
		std::cout << "---------------------" << '\n';
		std::cout << "signal generator parameter:" << '\n';
		auto name = wavename.find(parameter.waveshape);
		if (name!= wavename.end()){
			std::cout << "The wave shape is: " << name->second << '\n';
		}
		else {
			std::cout << "Not a valid wave shape \n";
		}
		std::cout << " Frequency: " << parameter.frequency << " Hz \n";
		std::cout << " Amplitude: " << parameter.amplitude << '\n';
		std::cout << " Phase: " << parameter.phase_shift << '\n';
		std::cout << " Number of Channels: " << parameter.num_of_channels << '\n';
	}

	void Source_SignalGenerator::DisplayInitialCondition()
	{
		// No initial condition for signal generator
	}

	Source_SignalGenerator::~Source_SignalGenerator()
	{
	}
}
