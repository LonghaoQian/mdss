/*
Aerodynamics angle calcuations:

input : 
0 rho
1 soundspeed
2 Vbx
3 Vby
4 Vbz
5 p
6 q
7 r
8 Ab_x // Find veloity derivative in body fixed frame :Vb_dot=Ab-wXVb
9 Ab_y // 
10 Ab_z // 
11 - 19 R_BI

output : 
0 TAS
1 MachNumber
2 alpha
3 beta
4 dynamic pressure
5 alpha_dot_bar
6 beta_dot_bar
7 p_bar
8 q_bar
9 r_bar
10 - 18 R_BW
*/
#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace aero {

	enum AeroAngleInput {
		AERO_INPUT_RHO= 0,
		AERO_INPUT_SOUNDSPEED,
		AERO_INPUT_Vbx,
		AERO_INPUT_Vby,
		AERO_INPUT_Vbz,
		AERO_INPUT_P,
		AERO_INPUT_Q,
		AERO_INPUT_R,
		AERO_INPUT_Ab_x,
		AERO_INPUT_Ab_y,
		AERO_INPUT_Ab_z
	};

	enum AeroAngleOutput {
		AERO_OUTPUT_TAS = 0,
		AERO_OUTPUT_MachNumber,
		AERO_OUTPUT_AOA,
		AERO_OUTPUT_SIDESLIP,
		AERO_OUTPUT_DYNAMICPRESSURE,
		AERO_OUTPUT_AOARATE, // will return the filtered result
		AERO_OUTPUT_SIDESLIPRATE, // will return the filtered result
		AERO_OUTPUT_Pbar,
		AERO_OUTPUT_Qbar,
		AERO_OUTPUT_Rbar
	};

	struct AeroAngleParameter {
		double min_airspeed_; // minimum airspeed for aero angle calculation 
		double b_; // wing span
		double c_bar_; // min chord
	};
	class AeroAngle :
		public Subsystem
	{
	private:
		AeroAngleParameter param_;
		Matrix3d R_BI;
		Vector3d omega_b;
		Vector3d Vb_dot;
		Vector3d Vb;
		double lon_normalizer;
		double lat_normalizer;
		double TAS_dot;
	public:
		AeroAngle(const AeroAngleParameter& parameter);
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


		~AeroAngle();
	};
}

