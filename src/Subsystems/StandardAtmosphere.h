#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 02 15

geographic libs contains the following blocks:

1. Standard Atomsphere
2. Gravity Model
TO DO :
3. Wind Gust Model
_________________________________
*/

#include "Subsystem.h"
#include "UtilityFunctions.h"

namespace geographic {
	// standard atmosphere
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
	// gravity model
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
