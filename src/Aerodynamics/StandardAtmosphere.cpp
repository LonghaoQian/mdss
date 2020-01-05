#include "pch.h"
#include "StandardAtmosphere.h"


StandardAtmosphere::StandardAtmosphere(const StandardAtmosphereParameter& parameter)
{
	system_info.system_type = ATOMSPHERE;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 1;
	system_info.num_of_outputs = 4;
	load_flag_ = atmosphere_file.ReadFromMatFile(atmosphere_data_, parameter.atmoshpere_name_, "atmosphere_data");
	if (load_flag_) {
		std::cout << "Atmosphere file has been successfully loaded! " << std::endl;
		// load lookup table data
		int num_of_cols = atmosphere_data_.cols();
		atmopshere_table_.LoadTableData(atmosphere_data_.col(0), atmosphere_data_.rightCols(num_of_cols - 1), false);
	}
	else {
		std::cout << "Failed to read data atmoshpere block will not run ! " << std::endl;
	}

	system_info.system_parameter_ok = 0;
	ready_to_run = true;

	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);

}

void StandardAtmosphere::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
}

void StandardAtmosphere::OutputEquation(const double & t,
										const VectorXd & state, 
										const VectorXd & input, 
										VectorXd & output)
{
	if (load_flag_) {
		atmopshere_table_.GetOutput(output, input(0)); // use the first element of the input at the target height
	}
}

void StandardAtmosphere::IncrementState()
{
}

void StandardAtmosphere::DisplayParameters()
{
	if (load_flag_) {
		std::cout << "Atmopshere file has been loaded!  " << std::endl;
	}
	else {
		std::cout << "No Atmosphere file loaded! " << std::endl;
	}
}

void StandardAtmosphere::DisplayInitialCondition()
{
	std::cout << "------No initial condition for atmopshere block----------" << std::endl;
}


StandardAtmosphere::~StandardAtmosphere()
{
}
