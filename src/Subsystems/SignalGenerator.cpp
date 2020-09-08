#include "pch.h"
#include "SignalGenerator.h"

namespace source_sink {
	PeriodicWave::PeriodicWave(const PeriodicWaveparameter& _parameter)
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

	void PeriodicWave::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
	{
		// No differential equation for perodic waves
	}

	void PeriodicWave::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
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
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = 2 * asin(temp) * parameter.amplitude /M_PI;
			}
			break;
		default: // If not specified, use sine wave
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = temp * parameter.amplitude;
			}
			break;
		}
	}

	void PeriodicWave::IncrementState()
	{
		// No increment state for signal generator
	}

	void PeriodicWave::DisplayParameters()
	{
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

	void PeriodicWave::DisplayInitialCondition()
	{
		// No initial condition for signal generator
	}

	PeriodicWave::~PeriodicWave()
	{
	}

	Step::Step(const Stepparameter & _parameter)
	{
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 0;
		system_info.type = source_STEP;
		system_info.num_of_outputs = _parameter.num_of_channels;
		parameter = _parameter;
		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);
	}
	void Step::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
	{
		// No differential function for step 
	}
	void Step::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		if (t>= parameter.steptime) {
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = parameter.value;
			}
		} else {
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = 0.0;
			}
		}
	}
	void Step::IncrementState()
	{
		// No increment state for step
	}
	void Step::DisplayParameters()
	{
		std::cout << "step parameter:" << '\n';
		std::cout << " Step Time: " << parameter.steptime << " s \n";
		std::cout << " Final Value: " << parameter.value << '\n';
	}
	void Step::DisplayInitialCondition()
	{
		// No initial condition for step
	}
	Step::~Step()
	{
	}
	Ramp::Ramp(const Rampparamter & parameter_)
	{
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 0;
		system_info.type = source_RAMP;
		system_info.num_of_outputs = parameter_.num_of_channels;
		parameter = parameter_;
		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);
	}
	void Ramp::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
	{
		// not differential equations for ramp
	}
	void Ramp::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		if (t >= parameter.starttime) {
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = parameter.rate * (t - parameter.starttime);
			}
		}
		else {
			for (int i = 0; i < system_info.num_of_outputs; i++) {
				output(i) = 0.0;
			}
		}
	}
	void Ramp::IncrementState()
	{
		// No increment state for ramp
	}
	void Ramp::DisplayParameters()
	{
	}
	void Ramp::DisplayInitialCondition()
	{
	}
	Ramp::~Ramp()
	{
	}
}
