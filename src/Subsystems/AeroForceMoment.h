#pragma once
/*
-------------------------------------------------------
Author: Longhao Qian 2020 Jan 6th
Subsystem for calculating aerodynamic forces and moments
Input: 
0.dynamic pressure
1.alpha
2.beta
3.p_bar
4.q_bar
5.r_bar
6.alpha_dot_bar
7.beta_dot_bar
8. flap
9. spoiler
10. airbreak

Output:
0-2:
F_B
3-5:
M_B
-------------------------------------------------------
*/
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace aero {

	/*
	Input:
	0.dynamic pressure
	1.  alpha
	2.  beta
	3.  p_bar
	4.  q_bar
	5.  r_bar
	6.  alpha_dot_bar
	7.  beta_dot_bar
	8.  flap
	9.  spoiler
	10. airbreak
	11. da
	12. de
	13. dr
	14. Mach number
	15. relative height
	16. reference_point_x
	17. reference_point_y
	18. reference_point_z
	*/

	enum aeroforceinput {
		AEROFORCE_INPUT_DYNAMICPRESSURE,
		AEROFORCE_INPUT_AOA,
		AEROFORCE_INPUT_SIDESLIP,
		AEROFORCE_INPUT_Pbar,
		AEROFORCE_INPUT_Qbar,
		AEROFORCE_INPUT_Rbar,
		AEROFORCE_INPUT_AOARATE_FILTERED,
		AEROFORCE_INPUT_SIDESLIPRATE_FILTERED,
		AEROFORCE_INPUT_FLAP,
		AEROFORCE_INPUT_SPOILER,
		AEROFORCE_INPUT_AIRBREAK,
		AEROFORCE_INPUT_AILERON,
		AEROFORCE_INPUT_ELEVATOR,
		AEROFORCE_INPUT_RUDDER,
		AEROFORCE_INPUT_MACHNUMBER,
		AEROFORCE_INPUT_RELATIVEHEIGHT,
		AEROFORCE_INPUT_RX,
		AEROFORCE_INPUT_RY,
		AEROFORCE_INPUT_RZ
	};

	/*
	Output:
	0-2:
	F_B
	3-5:
	M_B
	*/

	enum aeroforceoutput {
		AEROFORCE_OUTPUT_FBx = 0,
		AEROFORCE_OUTPUT_FBy,
		AEROFORCE_OUTPUT_FBz,
		AEROFORCE_OUTPUT_MBx,
		AEROFORCE_OUTPUT_MBy,
		AEROFORCE_OUTPUT_MBz
	};

	struct AerosForceParameter {
		double S;
		double b_;
		double c_bar_;
		struct {
			double CL0_;
			double CL_alpha_;
			double CL_alpha_squared_;
			double CL_alpha_cubed_;
			double CLde_;
			double CLadot_;
			double CLq_;

			double CD0_;
			double CD_alpha_;
			double CD_alpha_squared_;
			double CDde_;
			double CDDf_;
			double CDbeta_;
			double CDground_;

			double CYb_;
			double CYda_;
			double CYdr_;
			double CYp_;
			double CYr_;

			double Cmalpha_;
			double CmDf_;
			double Cmq_;
			double Cmadot_;
			double Cm0_;
			double Cmde_;

			double  Clb_;
			double  Clp_;
			double  Clr_;
			double  Clda_;
			double  Cldr_;

			double Cnb_;
			double Cnp_;
			double Cnr_;
			double Cnda_;
			double Cndr_;
		}AeroCoefficient;
	};
	class AeroForceMoment1 :
		public Subsystem
	{
	public:
		AeroForceMoment1(const AerosForceParameter& param);
		~AeroForceMoment1();
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
		AerosForceParameter param_;
		Matrix3d R_WB;
		Vector3d F_W;// aero force in wind asixs
		Vector3d M_B0; // momemt around base point
		Vector3d ReferencePoint; // the vector point from the center of mass to the aeroforce reference point
		Matrix3d refer_point_cross_;
		// temp variable 
		double QS; 
		double QScbar;
		double QSb;
		double NormalizedRelativeHeight;
		// total aerodynamic coefficients
		double CL_, CD_, CY_, Cl_, Cm_, Cn_; 
		void CalculateR_BW(const double& alpha_, const double& beta_);
	};

}
