#pragma once
/*
___________________________________________________________________________________________________
Author: Longhao Qian
Data:   2020 11 02

geographic libs contains the following blocks:

1. International Standard Atomsphere (ISA)
TO DO :
2. Non-standard atomsphere
3. Density Model based on outside atomsphere temperature (OAT) and pressure altitude

4. Gravity Model
TO DO :
3. Wind Gust Model (wind speed based on altitude and other parameters)
___________________________________________________________________________________________________
*/
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace geographic {
	// 1. standard atmosphere (ISA)
	enum AtmoshpereOutputIndex{
		AtmTemperature = 0,
		AtmSoundSpeed,
		AtmPressure, 
		AtmDensity
	};

	struct StandardAtmosphereParameter {
		const char* atmoshpere_name_;
	};
	class StandardAtmosphere :
		public Subsystem
	{
	private:
		// parameters
		MatrixXd atmosphere_data_;
		mathauxiliary::Lookup_1D atmopshere_table_;
	public:
		StandardAtmosphere(const StandardAtmosphereParameter& parameter);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~StandardAtmosphere();
	};
	// 2. Non -standard atmosphere

	// 3. Density Model
	struct DensityModelParameter {
		double Rg{ 287.058 }; // specific gas constant, the default is dry air
	};

	enum DensityModelinput {
		DENSITYMODEL_INPUT_TEMERATURE= 0, // input temperature (K)
		DENSITYMODEL_INPUT_PRESSURE,       // input pressure (pa)
	};

	enum DensityModeloutput {
		DENSITYMODEL_OUTPUT_DENSITY = 0,
	};

	class DensityModel :
		public Subsystem {
	public:
		DensityModel(const DensityModelParameter& parameter);
		~DensityModel();
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
	private:
		DensityModelParameter param;
	};


	// 4. gravity model 
	enum GravityMode
	{
		FlatGround, Sphere, Ellipsoid
	};
	struct GravityModelParameter {
		GravityMode Mode;
	};
	class Gravity :
		public Subsystem
	{
	private:
		// parameters
		GravityModelParameter param_;
		// RIB
	public:
		Gravity(const GravityModelParameter& parameter);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Gravity();
	};
}
