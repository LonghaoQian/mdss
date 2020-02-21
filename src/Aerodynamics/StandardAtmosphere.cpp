#include "pch.h"
#include "StandardAtmosphere.h"

namespace geographic {
	StandardAtmosphere::StandardAtmosphere(const StandardAtmosphereParameter& parameter)
	{
		system_info.system_type = ATOMSPHERE;
		system_info.type = geographic_ATOMSPHERE;
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
			atmopshere_table_.GetOutput(output, input(0)); // use the first element of the input as the target height
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
	/*_____________________________

	Gravity lib
	Coordinate frame: NED
	g = 9.81 at sea level
	_____________________________
	*/
	Gravity::Gravity(const GravityModelParameter& parameter)
	{
		system_info.system_type = GRAVITY;
		system_info.type = geographic_GRAVITY;
		param_ = parameter;	
		switch (param_.Mode) {
		case FlatGround:
			system_info.DIRECT_FEED_THROUGH = false;
			system_info.num_of_inputs = 0;
			break;
		case Ellipsoid:
			system_info.DIRECT_FEED_THROUGH = true;
			system_info.num_of_inputs = 3;// lat lon height
			break;
		case Sphere:
			system_info.DIRECT_FEED_THROUGH = true;
			system_info.num_of_inputs = 3;// lat lon height
			break;
		default:
			system_info.DIRECT_FEED_THROUGH = false;
			system_info.num_of_inputs = 0;// lat lon height
			break;
		}
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_outputs = 3;// gravity in inertial frame NED axis

		system_info.system_parameter_ok = 0;
		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);
		ready_to_run = true;
	}
	void Gravity::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// No differential equations for gravity block
	}
	void Gravity::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		switch (param_.Mode) {
		case FlatGround:
			output << 0, 0, 9.81;
			break;
		default:
			output << 0, 0, 9.81;
			break;
		}
	}
	void Gravity::IncrementState()
	{
		// No increment state for gravity 
	}
	void Gravity::DisplayParameters()
	{
		switch (param_.Mode) {
		case FlatGround:
			std::cout << "Flat Ground Mode " << std::endl;
			break;
		case Ellipsoid:
			std::cout << "Ellipsoid Mode " << std::endl;
			break;
		case Sphere:
			std::cout << "Sphere Mode " << std::endl;
			break;
		default:
			std::cout << "Flat Ground Mode " << std::endl;
			break;
		}
	}
	void Gravity::DisplayInitialCondition()
	{
		std::cout << "No initial condition for gravity block" << std::endl;
	}
	Gravity::~Gravity()
	{
	}
}