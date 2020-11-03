#pragma once
/*
___________________________________________________________________________________________________
Author: Longhao Qian
Data:   2020 08 11

propulsion blocks

1. CFM 56 N1, Fuel Flow and EGT
2. propeller model simple 
3. Propeller Model  Fixed Pitch
4. propeller model  variable pitch
5. Piston Engine Model
6. Brush electric motor model simple
_____________________________________________________________________________________________________
*/
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace propulsionsystem {
	/*----------------------- a CFM 56 engine model ------------------------ */
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
	/*----------------------- propeller simple fixed pitch ------------------------ */

	struct SimplePropellerParameter {
		double minimumAngularRate; // minimum angluar velocity to produce lift (RPS)
		double MomentCoefficient;  // the drag moment generated by propeller
		double ThrustCoefficient;  // the thrust vs auglar velocity 
		bool   SpinningDirection;  // spinning direction viewed from the front of the propeller, + : counter clock wise, -: clock wise
	};

	// TO DO: add the simple propeller model T = k omega^2

	/*----------------------- propeller model based on chart ------------------------ */
	enum PropellerChartInput {
		PROPELLER_INPUT_N = 0,  // the input propeller RPS (rounds per second) from the sharft 
		PROPELLER_INPUT_V,      // the forward velocity of the propeller (m/s)
		PROPELLER_INPUT_RHO,    // the air density
		PROPELLER_INPUT_PITCH   // the pitch of the propeller (only used in variable propeller model)
	};
	enum PropellerChartOutput {
		PROPELLER_OUTPUT_T = 0,// the output thrust
		PROPELLER_OUTPUT_Q,    // the output required torque
	};
	/*----------------------- propeller chart fixed pitch------------------------ */
	struct PropellerChartFixedPitchParameter {
		Matrix<double, Eigen::Dynamic, 3> Chart; // an N by 3 matrix , first col: J, second col: CT, third col: CP
		double minimumAngularRate;               // the minimum Augular rate, if below this threshold, the propeller generates no aerdynamics forces and power consumptions
		double diameter;                         // the diameter of the propeller (m) 
	};

	class PropellerChartFixedPitch :
		public Subsystem 
	{
	private:
		PropellerChartFixedPitchParameter parameter;
		double J;  // the advance ratio
		double D_4;// power of 4 of disc diameter
		double D_5;// power of 5 of disc diameter
		double N_2; // square of rotor speed
		VectorXd Coefficient; // CT  = 0, CP = 1
		mathauxiliary::Lookup_1D propeller_fixed_pitch_;
	public:
		PropellerChartFixedPitch(const PropellerChartFixedPitchParameter& param);
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
		~PropellerChartFixedPitch();
	};
	/*----------------------- propeller chart variable pitch------------------------ */
	struct PropellerChartVariablePitchParameter {
		MatrixXd Chart_T;
		MatrixXd Chart_P;
		VectorXd Jindex;
		VectorXd PitchIndex;
		double minimumAngularRate;
		double diameter;
	};

	class PropellerChartVariablePitch :
		public Subsystem
	{
	private:
		PropellerChartVariablePitchParameter parameter;
		double J;
		double CT;
		double CP;
		mathauxiliary::Lookup_2D propeller_table_T_;
		mathauxiliary::Lookup_2D propeller_table_P_;
		double D_4;// power of 4 of disc diameter
		double D_5;// power of 5 of disc diameter
		double N_2; // square of rotor speed
	public:
		PropellerChartVariablePitch(const PropellerChartVariablePitchParameter& param);
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
		~PropellerChartVariablePitch();
	};
	/*----------------------- piston engine ------------------------ */
	enum PistonEngineInput {
		PISTONENGINE_INPUT_THROTTLE = 0,  // normalized throttle 0 - 1
		PISTONENGINE_INPUT_MIXTURE,       // normalized mixture  -1 - 1
		PISTONENGINE_INPUT_SHAFTRPS,      // RPS of shaft 
		PISTONENGINE_INPUT_MANIFOLD,      // air density at manifold
		PISTONENGINE_INPUT_STARTER,       // turn on the starter (>0.0) means the starter is on
		PISTONENGINE_INPUT_FUELSTATE,     // whether fuel is supplied (>0.0) means supplied
		PISTONENGINE_INPUT_SUPERCHARGER,  // if the supercharger is on (>0.0) means the supercharger is on
	};

	enum PistonEngineState {
		PISTONENGINE_STATE_OFF= 0,
		PISTONENGINE_STATE_ON,
	};

	enum PistonEngineOutput {
		PISTONENGINE_OUTPUT_Q = 0, // output torque
		PISTONENGINE_OUTPUT_FUELRATE,
	};
	/*-------- piston engine model------- */
	struct PistonEngineParameter {
		Eigen::Matrix<double, Dynamic, 2> TorqueRPMChart;                         // output power versus RPM chart  an N by 2 matrix 0 RPM 1 Torque (m*s)
		Eigen::Matrix<double, Dynamic, 2> PowerMixtureChart;                     // power factor versus mixture chart an N by 2 matrix  0 RPM  1 toque factor
		Eigen::Matrix<double, Dynamic, 2> MixturePowerFactorSFCfactorChart;    // sfc factor versus mixture chart   an N by 2 matrix 0 RPM  1 SFC factor
		double idle_RPM;                  // PRM 
		double shaft_damping;             // shaft damping below the idle RPM
		double sfc;						  // specific fuel consumption LB/(BHP*HR) pounds of fuel per break horse power per hour
		double superchargerfactor;        // the power factor by turnning on super charger
		double krho0;                     // rate parameter for manifold density amplifier 
		double krho1;					  // base parameter for manifold density amplifier  
		double stater_max_torque;         // the maximum torque the starter could provide 
		double stater_breakaway_RPM;      // the PRM where the starter torque begins to decline with RPM
		double stater_zero_torque_RPM;    // if below this torque, the engine produces no torque output
	};

	class PistonEngine :
		public Subsystem
	{
	private:
		PistonEngineParameter parameter;
		double RPM;
		double EngineON;
		double starter_torque;
		double throttle;
		double starter_torque_slope;
		double fuel_normal;
		double density_factor;
		mathauxiliary::Lookup_1D rpm_torque_;
		mathauxiliary::Lookup_1D mixture_powerfactor_;
		mathauxiliary::Lookup_1D mixture_sfcfactor_;
	public:
		PistonEngine(const PistonEngineParameter& param);
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
		~PistonEngine();
	};
	// TO DO: add electric motor:

	class BrushSimpleElectricMotor :
		public Subsystem
	{
	private:

	public:

	};
}