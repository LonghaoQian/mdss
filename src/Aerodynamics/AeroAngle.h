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
		rho = 0,
		soundspeed,
		Vbx,
		Vby,
		Vbz,
		p,
		q,
		r,
		Ab_x,
		Ab_y, 
		Ab_z 
	};

	enum AeroAngleOutput {
		TAS = 0,
		MachNumber,
		alpha,
		beta,
		dynamic_pressure,
		alpha_dot_bar,
		beta_dot_bar,
		p_bar,
		q_bar,
		r_bar
	};

	struct AeroAngleParameter {
		double min_airspeed_;
		double b_;
		double c_bar_;
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

