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
#include <iostream>
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace aero {

	enum aeroinput {
		alpha,
		beta,
		p_bar,
		q_bar,
		r_bar,
		alpha_dot_bar,
		beta_dot_bar,
		flap,
		spoiler,
		airbreak
	};

	struct AeroDerivative {

	};

	struct Aeroparameter{
		double CL0_;
		mathauxiliary::Lookup_2D  CL_alpha_beta;
		mathauxiliary::Linear     CL_M;
		mathauxiliary::Lookup_1D  CLDf_;
		mathauxiliary::Linear     CLde_;
		mathauxiliary::Linear     CLadot_;
		mathauxiliary::Lookup_1D  CLground_;

		double CD0_;
		mathauxiliary::Lookup_2D  CD_alpha_flap_;
		mathauxiliary::Linear     CDde_;
		mathauxiliary::Lookup_1D  CDDf_;
		mathauxiliary::Linear     CDbeta_;
		mathauxiliary::Lookup_1D  CDground_;

		mathauxiliary::Linear     CYb_;
		mathauxiliary::Linear     CYda_;
		mathauxiliary::Linear     CYdr_;
		mathauxiliary::Linear     CYp_;
		mathauxiliary::Linear     CYr_;

		mathauxiliary::Linear     Cmalpha_;
		mathauxiliary::Lookup_1D  CmDf_;
		mathauxiliary::Linear     Cmq_;
		mathauxiliary::Linear     Cmadot_;
		double                    Cm0_;
		mathauxiliary::Linear     Cmde_;

		mathauxiliary::Linear     Clb_;
		mathauxiliary::Linear     Clp_;
		mathauxiliary::Lookup_1D  Clr_alpha_;
		mathauxiliary::Linear     Clda_;
		mathauxiliary::Linear     Cldr_;

		mathauxiliary::Linear     Cnb_;
		mathauxiliary::Linear     Cnp_;
		mathauxiliary::Linear     Cnr_;
		mathauxiliary::Linear     Cnda_;
		mathauxiliary::Linear     Cndr_;
	};
	
	struct AerosForceParameter {
		double S;
		double b_;
		double c_bar_;
		Vector3d aero_reference_point_;// reference point from 
		Aeroparameter aeroparam_;
	};
	class AeroForceMoment1 :
		public Subsystem
	{
	private:
		AerosForceParameter param_;
		Matrix3d R_WB;
		Vector3d F_W;// aero force in wind asixs
		Vector3d M_B0; // momemt around base point
		Matrix3d refer_point_cross_;
		double CL_, CD_, CY_, Cl_, Cm_, Cn_; // total aerodynamic coefficients
		void CalculateR_BW(const double& alpha_, const double& beta_);
	public:
		AeroForceMoment1(const AerosForceParameter& param);
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
		~AeroForceMoment1();
	};

#ifdef USE_MATLAB_DEBUG
	bool LoadAeroParameter(const char* filename, const );
#endif // USE_MATLAB_DEBUG
}
