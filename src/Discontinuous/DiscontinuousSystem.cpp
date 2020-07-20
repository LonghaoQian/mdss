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
					param_.lower_bound,
					param_.lower_bound);
				
			}
			break;
		default:// both
			for (int i = 0; i < param_.num_of_channels; i++) {
				output(i) = mathauxiliary::SaturationElementalWise(input(i),
					param_.lower_bound,
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
	}

	void Switch::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
	}

	void Switch::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
	}

	void Switch::IncrementState()
	{
	}

	void Switch::DisplayParameters()
	{
	}

	void Switch::DisplayInitialCondition()
	{
	}

	Switch::~Switch()
	{
	}

}