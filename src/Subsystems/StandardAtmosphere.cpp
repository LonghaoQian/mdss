#include "pch.h"
#include "StandardAtmosphere.h"

namespace geographic {
	StandardAtmosphere::StandardAtmosphere(const StandardAtmosphereParameter& parameter)
	{
		system_info.type = geographic_ATOMSPHERE;
		system_info.category = GEOGRAPHIC;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 1;
		system_info.num_of_outputs = 4;
		
		atmosphere_data_.resize(44, 5);
		// load the atmosphere data h, T, a, P, rho
		atmosphere_data_ << 0, 288.2, 340.3, 1.0133, 1.225,
			100, 287.6, 340.0, 0.9794, 1.187,
			500, 284.9, 338.4, 0.95461, 1.167,
			1000, 281.7, 336.4, 0.89876, 1.111,
			1500, 278.2, 334.5, 0.84560, 1.058,
			2000, 275.2, 332.5, 0.79501, 1.007,
			2500, 271.9, 330.6, 0.74692, 0.9570,
			3000, 268.7, 328.6, 0.70121, 0.9093,
			3500, 265.4, 326.6, 0.65780, 0.8634,
			4000, 262.2, 324.6, 0.61660, 0.8194,
			4500, 258.9, 322.6, 0.57753, 0.7770,
			5000, 255.7, 320.5, 0.54048, 0.7364,
			5500, 252.4, 318.5, 0.50539, 0.6975,
			6000, 249.2, 316.5, 0.47218, 0.6601,
			6500, 245.9, 314.4, 0.44075, 0.6243,
			7000, 242.7, 312.3, 0.41105, 0.5900,
			7500, 239.5, 310.2, 0.38300, 0.5572,
			8000, 236.2, 308.1, 0.35652, 0.5258,
			8500, 233.0, 306.0, 0.33154, 0.4958,
			9000, 229.7, 303.8, 0.30801, 0.4671,
			9500, 226.5, 301.7, 0.28585, 0.4397,
			10000, 223.3, 299.5, 0.26500, 0.4135,
			11000, 216.7, 295.1, 0.22700, 0.3648,
			12000, 216.7, 295.1, 0.19399, 0.3119,
			13000, 216.7, 295.1, 0.16580, 0.2666,
			14000, 216.7, 295.1, 0.14170, 0.2279,
			15000, 216.7, 295.1, 0.12112, 0.1948,
			16000, 216.7, 295.1, 0.10353, 0.1665,
			17000, 216.7, 295.1, 0.088497, 0.1432,
			18000, 216.7, 295.1, 0.075652, 0.1217,
			19000, 216.7, 295.1, 0.064675, 0.1040,
			20000, 216.7, 295.1, 0.055293, 0.08891,
			25000, 221.5, 298.4, 0.025492, 0.04008,
			30000, 226.5, 301.7, 0.011970, 0.01841,
			35000, 236.5, 308.3, 0.0057459, 0.008463,
			40000, 250.4, 317.2, 0.0028714, 0.003996,
			45000, 264.2, 325.3, 0.0014910, 0.001966,
			50000, 270.7, 329.8, 0.00079779, 0.001027,
			55000, 265.6, 326.7, 0.000427516, 0.0005608,
			60000, 255.8, 320.6, 0.00022461, 0.0003059,
			65000, 239.3, 310.1, 0.00011446, 0.0001667,
			70000, 219.7, 297.1, 0.000055205, 0.0000875,
			75000, 200.2, 283.6, 0.000024904, 0.0000434,
			80000, 180.7, 269.4, 0.000010366, 0.00002;

		int num_of_cols = atmosphere_data_.cols();
		atmopshere_table_.LoadTableData(atmosphere_data_.col(0), atmosphere_data_.rightCols(num_of_cols - 1), false);

		system_info.system_parameter_ok = 0;
		ready_to_run = true;

		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);

	}

	void StandardAtmosphere::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no differential equation for atmosphere block
	}

	void StandardAtmosphere::OutputEquation(const double & t,
		const VectorXd & state,
		const VectorXd & input,
		VectorXd & output)
	{
			atmopshere_table_.GetOutput(output, input(0)); // use the first element of the input as the target height
	}

	void StandardAtmosphere::IncrementState()
	{
		// no increment state for standard atmosphere block
	}

	void StandardAtmosphere::DisplayParameters()
	{
		std::cout << "Atmopshere file has been loaded!  " << std::endl;
	}

	void StandardAtmosphere::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for atmopshere block----------" << std::endl;
	}

	/******************************************************************************************************/

	DensityModel::DensityModel(const DensityModelParameter & parameter) {
		system_info.type = geographic_DENSITYMODEL;
		system_info.category = GEOGRAPHIC;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 2;
		system_info.num_of_outputs = 1;

		system_info.system_parameter_ok = 0;
		ready_to_run = true;

		output.resize(system_info.num_of_outputs);
		output.setZero(system_info.num_of_outputs);
		param = parameter;
	}
	DensityModel::~DensityModel()
	{
	}
	void DensityModel::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no differential equations for density model
	}
	void DensityModel::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		output(DENSITYMODEL_OUTPUT_DENSITY) = input(DENSITYMODEL_INPUT_PRESSURE) / (param.Rg*input(DENSITYMODEL_INPUT_TEMERATURE));
	}
	void DensityModel::IncrementState()
	{
		// no increment state for density model
	}
	void DensityModel::DisplayParameters()
	{
		std::cout << "Parameter for the density model is: " << std::endl;
		std::cout << "Rg: " << param.Rg << " J/(kg k) \n";
	}
	void DensityModel::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for density model block----------" << std::endl;
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