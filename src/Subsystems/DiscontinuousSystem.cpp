#include "pch.h"
#include "DiscontinuousSystem.h"
namespace discontinuoussystem {

	Saturation::Saturation(const SaturationParameter & param)
	{
		system_info.type = discontinuous_SATURATION;
		system_info.category = DISCONTINUOUS;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;

		system_info.system_parameter_ok = 0;
		ready_to_run = true;
		param_ = param;

		system_info.num_of_inputs = param_.num_of_channels;
		system_info.num_of_outputs = param_.num_of_channels;
		output.resize(system_info.num_of_outputs);
		output.setZero();

	}

	void Saturation::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// No differential equations for saturation block
	}

	void Saturation::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		switch (param_.type)
		{
		case SATURATION_UPPER:
			for (int i = 0; i < param_.num_of_channels; i++) {
				if (input(i) >= param_.upper_bound) {
					output(i) = param_.upper_bound;
				}
			}
			break;
		case SATURATION_LOWER:
			for (int i = 0; i < param_.num_of_channels; i++) {
				if (input(i) <= param_.lower_bound) {
					output(i) = param_.lower_bound;
				}
			}
			break;
		case SATURATION_BOTH:
			for (int i = 0; i < param_.num_of_channels; i++) {
				output(i) = mathauxiliary::SaturationElementalWise(input(i),
					param_.upper_bound,
					param_.lower_bound);
				
			}
			break;
		default:// both
			for (int i = 0; i < param_.num_of_channels; i++) {
				output(i) = mathauxiliary::SaturationElementalWise(input(i),
					param_.upper_bound,
					param_.lower_bound);
			}
			break;
		}
	}

	void Saturation::IncrementState()
	{
		// No increment state for saturation block
	}

	void Saturation::DisplayParameters()
	{
		std::cout << "saturation type is: ";
		switch (param_.type)
		{
		case SATURATION_UPPER:
			std::cout << "upper" << std::endl;
			break;
		case SATURATION_LOWER:
			std::cout << "lower" << std::endl;
			break;
		case SATURATION_BOTH:
			std::cout << "both" << std::endl;
			break;
		default:// both
			std::cout << "both" << std::endl;
			break;
		}
		std::cout << "upper limit is: " << param_.upper_bound << " lower limit is: " << param_.lower_bound << std::endl;
		std::cout << "number of channels is: " << param_.num_of_channels << std::endl;
	}

	void Saturation::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for saturation ----------" << std::endl;
	}

	Saturation::~Saturation()
	{
	}

	Switch::Switch(const SwitchParameter & param)
	{
		system_info.type = discontinuous_SWITCH;
		system_info.category = DISCONTINUOUS;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;

		system_info.system_parameter_ok = 0;
		ready_to_run = true;
		param_ = param;

		system_info.num_of_inputs = 1+2*param_.num_of_channels;
		system_info.num_of_outputs = param_.num_of_channels;
		output.resize(system_info.num_of_outputs);
		output.setZero();
	}

	void Switch::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// No differential equations for switch block
	}

	void Switch::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		// if input>switch value, use input(1 -> param_.num_of_channels ), else input(param_.num_of_channels + 1 -> 2* param_.num_of_channels)
		if (input(SWITCH_INPUT) > param_.switch_value) {
			output = input.segment(1, param_.num_of_channels);
		} else {
			output = input.segment(param_.num_of_channels + 1, param_.num_of_channels);
		}
	}

	void Switch::IncrementState()
	{
		// No incrementstate switch block
	}

	void Switch::DisplayParameters()
	{
		std::cout << "switch number : " << param_.switch_value  << std::endl;
		std::cout << "number of channels is: " << param_.num_of_channels << std::endl;
	}

	void Switch::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for switch ----------" << std::endl;
	}

	Switch::~Switch()
	{
	}

}