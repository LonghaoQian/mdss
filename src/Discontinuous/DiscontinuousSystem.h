#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 07 19

discontinuos blocks contains the following blocks:

1. Saturtion block
2. Switch block
TO DO:
3. Backlash block
4. Coulomb and Viscous Friction
5. Wrap to 0
6. Rate Limiter
_________________________________
*/

#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace discontinuoussystem {

	enum SaturationType{
		SATURATION_UPPER,
		SATURATION_LOWER,
		SATURATION_BOTH
	};

	struct SaturationParameter {
		int num_of_channels;
		double upper_bound;
		double lower_bound;
		SaturationType type;
	};	

	class Saturation :
		public Subsystem
	{
	private:
		SaturationParameter param_;
	public:
		Saturation(const SaturationParameter& param);
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
		~Saturation();
	};
	//--------------------------------------------------
	// 

	enum {
		SWITCH_INPUT = 0
	};

	struct SwitchParameter {
		int num_of_channels;
		double switch_value;
	};
	class Switch :
		public Subsystem
	{
	private:
		SwitchParameter param_;
	public:
		Switch(const SwitchParameter& param);
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
		~Switch();
	};
}