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
		AEROFORCE_INPUT_RX,                      // distance from the center of mass of the plane to the aerodynamics reference point
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
		AEROFORCE_OUTPUT_MBz,
		AEROFORCE_OUTPUT_R_WB00,
		AEROFORCE_OUTPUT_R_WB10,
		AEROFORCE_OUTPUT_R_WB20,
		AEROFORCE_OUTPUT_R_WB01,
		AEROFORCE_OUTPUT_R_WB11,
		AEROFORCE_OUTPUT_R_WB21,
		AEROFORCE_OUTPUT_R_WB02,
		AEROFORCE_OUTPUT_R_WB12,
		AEROFORCE_OUTPUT_R_WB22,
		AEROFORCE_OUTPUT_LIFT,
		AEROFORCE_OUTPUT_DRAG,
		AEROFORCE_OUTPUT_SIDE,
	};

	struct AerosForceParameter {
		double S;
		double b_;
		double c_bar_;
		struct {
			struct {
				double CL0_{0.0};
				double CL_alpha_{ 0.0 };
				double CL_alpha_squared_{ 0.0 };
				double CL_alpha_cubed_{ 0.0 };
				double CLde_{ 0.0 };
				double CLadot_{ 0.0 };
				double CLq_{ 0.0 };
				double CL_flap_{ 0.0 };
				double CL_flap_squared_{ 0.0 };
			} Lift;

			struct {
				double CD0_{ 0.0 };
				double CD_alpha_{ 0.0 };
				double CD_alpha_squared_{ 0.0 };
				double CDde_{ 0.0 };
				double CD_flap_{ 0.0 };
				double CD_flap_squared_{ 0.0 };
				double CDbeta_{ 0.0 };
				double CDground_{ 0.0 };
			} Drag;

			struct 
			{
				double CYb_{ 0.0 };
				double CYda_{ 0.0 };
				double CYdr_{ 0.0 };
				double CYp_{ 0.0 };
				double CYr_{ 0.0 };
			} Side;

			struct {
				double  Clb_{ 0.0 };
				double  Clp_{ 0.0 };
				double  Clr_{ 0.0 };
				double  Clda_{ 0.0 };
				double  Cldr_{ 0.0 };
			}Roll;

			struct {
				double Cmalpha_{ 0.0 };
				double Cm_flap_{ 0.0 };
				double Cm_flap_squared_{ 0.0 };
				double Cmq_{ 0.0 };
				double Cmadot_{ 0.0 };
				double Cm0_{ 0.0 };
				double Cmde_{ 0.0 };
			} Pitch;

			struct {
				double Cnb_{ 0.0 };
				double Cnp_{ 0.0 };
				double Cnr_{ 0.0 };
				double Cnda_{ 0.0 };
				double Cndr_{ 0.0 };
			} Yaw;
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
		double AOA_square;
		double AOA_cube;
		double flap_square;
		double QS; 
		double QScbar;
		double QSb;
		double NormalizedRelativeHeight;
		// total aerodynamic coefficients
		double CL_, CD_, CY_, Cl_, Cm_, Cn_; 
		void CalculateR_BW(const double& alpha_, const double& beta_);
	};

}
