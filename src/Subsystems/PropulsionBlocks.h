#pragma once
/*
_________________________________
Author: Longhao Qian
Data:   2020 08 11

propulsion blocks

1. CFM 56 N1, Fuel Flow and EGT
2. Propeller Model (Propeller Chart)
3. Piston Engine Model
__________________
*/
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace propulsionsystem {

	enum CFM56modelOutput {
		CFM56_OUTPUT_N1 = 0,
		CFM56_OUTPUT_EGT,
		CFM56_OUTPUT_FF
	};

	struct CFM56Parameter {

		struct  {
			double Tf;
			double b0;
			double b1;
			double b2;
			double k0;
			double power0;
			double k1;
			double c1;
			double k2;
			double c2;
		}CFM56N1model;

		struct {
			double b0;
			double k0;
			double b1;
			double k1;
			double c1;
			double environment_temp;
			double Tf;
		}CFM56EGTmodel;

		struct {
			double b0;
			double b1;
			double k0;
			double k1;
			double c0;
			double c1;
			double Tf;
			double d1;
			double power1;
		}CFM56FuelFlowmodel;
	};

	class CFM56AuxiliaryModel :
		public Subsystem
	{
	private:
		CFM56Parameter parameter;
		double N2_d;
		double N2_c;
		double invN1Tf;
		double invEGTTf;
		double invFFTf;
		double N1_input;
		double EGT_input;
		double FF_input;
		mathauxiliary::Lookup_2D Max;
		mathauxiliary::Lookup_2D Idel;
		double NormalizedIdleThrust;
		double NormalizedMaxThrust;
	public:
		CFM56AuxiliaryModel(const CFM56Parameter& param);
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
		~CFM56AuxiliaryModel();
	};

	enum CFM56ThrustInput {
		CFM56_INPUT_N1 = 0,
		CFM56_INPUT_Mach,
		CFM56_INPUT_Height
	};

	struct CF56ThrustModelParameter {
		VectorXd Mach;
		VectorXd Height;
		MatrixXd Max;
		MatrixXd Idle;
		double MaxThrust;
		double IdelN1;
		double MaxN1;
	};

	class CFM56ThrustModel :
		public Subsystem
	{
	private:
		CF56ThrustModelParameter parameter;
		double N1_dff;
		double Idle;
		double Max;
		mathauxiliary::Lookup_2D MaxLookUp;
		mathauxiliary::Lookup_2D IdelLookUp;
	public:
		CFM56ThrustModel(const CF56ThrustModelParameter& param);
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
		~CFM56ThrustModel();
	};
	/*
	class PropellerChart :
		public Subsystem 
	{
	private:
	public:

	};
	
	class PistonEngine :
	public Subsystem
	{

	}

	
	*/
}