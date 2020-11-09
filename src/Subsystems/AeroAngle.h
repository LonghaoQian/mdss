/*
___________________________________________________________________________________________________
Author: Longhao Qian
Data:   2020 11 01
1. Aerodynamics angle calculation
2. Convert true airspeed to calibrated airspeed
___________________________________________________________________________________________________
*/
#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"

namespace aero {

	const double a0{ 340.6520138 }; // standard sound speed at sea level and at 15 celsius (m/s)
	const double P0{ 101325 }; // standard pressure at sea level (P)

	enum AeroAngleInput {
		AERO_INPUT_RHO= 0,               // the air density
		AERO_INPUT_SOUNDSPEED,           // the sound speed
		AERO_INPUT_Vbx,                  // aircraft inertial velocity in the body-fixed frame
		AERO_INPUT_Vby,
		AERO_INPUT_Vbz,
		AERO_INPUT_P,                    // aircraft angular velocity in the body-fixed frame
		AERO_INPUT_Q,          
		AERO_INPUT_R,
		AERO_INPUT_Vbdotx,               // aircraft inertia acc in the body-fixed frame
		AERO_INPUT_Vbdoty,
		AERO_INPUT_Vbdotz,
		AERO_INPUT_HORIZONTALWINDSPEED,  // magnitude of windspeed in (m/s) always positive
		AERO_INPUT_WINDDIRECTION,        // the direction where the wind is blowing from, i.e. the opposite direction of where the wind travels.
		AERO_INPUT_VERTICALWINDSPEED,    // the vertical speed of the wind, positive means upwards, and negative means downwards.
		AERO_INPUT_R_BI00,               // the rotation matrix between the body-fixed frame and inertial frame
		AERO_INPUT_R_BI10,
		AERO_INPUT_R_BI20,
		AERO_INPUT_R_BI01,
		AERO_INPUT_R_BI11,
		AERO_INPUT_R_BI21,
		AERO_INPUT_R_BI02,
		AERO_INPUT_R_BI12,
		AERO_INPUT_R_BI22,
		AERO_INPUT_VIx,                 // aircraft speed in the inertial frame (for gamma calculation)
		AERO_INPUT_VIy,
		AERO_INPUT_VIz,
	};

	enum AeroAngleOutput {
		AERO_OUTPUT_TAS = 0,
		AERO_OUTPUT_MACHNUMBER,
		AERO_OUTPUT_AOA,
		AERO_OUTPUT_SIDESLIP,
		AERO_OUTPUT_DYNAMICPRESSURE,
		AERO_OUTPUT_AOARATE,          // will return the filtered result
		AERO_OUTPUT_SIDESLIPRATE,     // will return the filtered result
		AERO_OUTPUT_Pbar,
		AERO_OUTPUT_Qbar,
		AERO_OUTPUT_Rbar,
		AERO_OUTPUT_GAMMA,            // flight path angle relative to the ground
	};

	struct AeroAngleParameter {
		double min_airspeed_;  // minimum airspeed for aero angle calculation 
		double b_;			   // wing span
		double c_bar_;		   // min chord
	};

	/*
		1. The aeroangle block calculates the aerodynamics angles and other important outputs for 
		calculating the aerodynamics forces.
	
	*/

	class AeroAngle :
		public Subsystem
	{
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
	private:
		AeroAngleParameter param_;
		Matrix3d R_BI;
		Vector3d Vb_dot;
		Vector3d Vb;
		Vector3d WindSpeed;
		Vector3d WindSpeedBody;
		Vector3d TASBody;
		double lon_normalizer;
		double lat_normalizer;
		double TAS;
		double TAS_cal;
		double TAS_dot;
		double TempCal_1;
		double TempCal_2;
	};

	struct CASConversionParameter {
		int maxiteration{ 20 };// maximum iteration steps for supersonic calculation
	};


	enum CASConversionInput {
		TAS2CAS_INPUT_MACHNUMBER = 0,        // mach number
		TAS2CAS_INPUT_STATICPRESSURE,		 // static pressure

	};

	enum CASConversionOutput {
		TAS2CAS_OUTPUT_CAS = 0,             // the calibrated airspeed
		TAS2CAS_OUTPUT_IMPACTPRESSURE,      // the impact pressure
	};

	/*
		2. The CAS block calculates the calibrated airspeed based on the machnumber and static pressure

	*/

	class CAS :
		public Subsystem
	{
	public:
		CAS(const CASConversionParameter& parameter);
		~CAS();
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
		CASConversionParameter param;
		double qc;
	};

}

