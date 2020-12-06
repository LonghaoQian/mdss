/*
___________________________________________________________________________________________________
Author: Longhao Qian
Data:   2020 11 12

the wrapper class for the fixed-wing aircraft simulation 
___________________________________________________________________________________________________
*/
#pragma once
#include "SimController.h"
namespace aircraft {

	const double gravityacc{9.81};

	enum autopliotpitchmode {
		PITCHMODE_DIRECT = 0,  // the pitch CAS is off
		PITCHMODE_CAS,       // the pitch CAS is on (C star controller). Normal Acc Command Mode
		PITCHMODE_GAMMA,     // flight path hold mode
		PITCHMODE_ALTITUDE,  // altitude command mode
	};

	enum autopliotrollhmode {
		ROLLMODE_DIRECT = 0, // the roll CAS is off
		ROLLMODE_CAS,      // the roll CAS is on
		ROLLMODE_ROLLANGLE,
		ROLLMODE_HEADINGANGLE,
	};

	struct C172input {
		struct {
			bool starter;    // starter true: ON
			double throttle; // normalized throttle 0-1 
			double mixture;  // normalized mixtue position - 1 1
		}engine;
		struct {
			double elevator;     // normalized elevator -1 1
			double aileron;      // normalized aileron  -1 1
			double rudder;       // normalized rudder   -1 1
			double elevatortrim; // elevatortrim        deg
			double flap;         // normalized flap     0 - 1
		}controlsurface;
		struct {
			double steering; // deg steering angle
			bool gearbreak;
			bool geardown; // gear force enable
		}gear;


		struct {
			bool autopilotmaster;   // master swith of the entire autopilot system
			struct {
				autopliotpitchmode mode{ PITCHMODE_DIRECT };
				double commandaltitude{0.0};
				double commandgamma{0.0};
			}pitchCAS;

			struct {
				autopliotrollhmode mode{ ROLLMODE_DIRECT };
				double commandrollangle{ 0.0 };
				double commandheadingangle{ 0.0 };
			}rollCAS;

			struct {
				bool ON;
				double targetspeed{0.0}; // target true airspeed (m/s)
				double trimthrottle{0.0}; // the trim throttle to the 
			}autothrottle;

		}autopilot;
	};

	struct aerodynamicsparameter {
		double MinAirspeed;
		struct {
			double CL0{ 0.0 };
			double CLalpha{0.0};
			double CLalpha_squared{ 0.0 };
			double CLalpha_cubed{ 0.0 };
			double CLalpha_dot{ 0.0 };
			double CLq{ 0.0 };
			double CLde{ 0.0 };
			double CLflap{ 0.0 };
			double CLflap_squared{ 0.0 };
			double CLalphaflap{ 0.0 };
		}lift;
		struct {
			double CYb{ 0.0 };
			double CYda{ 0.0 };
			double CYdr{ 0.0 };
			double CYp{ 0.0 };
			double CYr{ 0.0 };
		}side;
		struct {
			double CD0_{ 0.0 };
			double CD_alpha_{ 0.0 };
			double CD_alpha_squared_{ 0.0 };
			double CDde_{ 0.0 };
			double CD_flap_{ 0.0 };
			double CD_flap_squared_{ 0.0 };
			double CDbeta_{ 0.0 };
			double CDground_{ 0.0 };
		}drag;

		struct {
			double  Clb_{ 0.0 };
			double  Clp_{ 0.0 };
			double  Clr_{ 0.0 };
			double  Clda_{ 0.0 };
			double  Cldr_{ 0.0 };
		}roll;

		struct {
			double Cmalpha_{ 0.0 };
			double Cm_flap_{ 0.0 };
			double Cm_flap_squared_{ 0.0 };
			double Cmq_{ 0.0 };
			double Cmadot_{ 0.0 };
			double Cm0_{ 0.0 };
			double Cmde_{ 0.0 };
		}pitch;

		struct {
			double Cnb_{ 0.0 };
			double Cnp_{ 0.0 };
			double Cnr_{ 0.0 };
			double Cnda_{ 0.0 };
			double Cndr_{ 0.0 };
		}yaw;

	};


	struct gearparameter {
		double Rex{ 0.0 };// gear position
		double Rey{ 0.0 };
		double Rez{ 0.0 };
		double Nex{ 0.0 };// gear direction
		double Ney{ 0.0 };
		double Nez{ 0.0 };
		double Stiffness{ 0.0 };
		double CompressDamping{ 0.0 };
		double ReboundDamping{ 0.0 };
		double Hmax{ 10.0 };
		double NeMin{ 0.1 };
		double VrelaxationRoll{0.0};
		double VrelaxationSide{ 0.0 };
		double Vlimit{ 0.0 };
		double SigmaDynamic{ 0.0 };
		double SigmaStatic{ 0.0 };
		double SigmaRoll{ 0.0 };
		double Sigma0{ 0.0 };
	};


	struct autopilotparameter {
		struct {
			double AltitudeErrorLimit{ 0.0 };
			double AltitudeErrorToVSGain{ 0.0 };
			double VSGain{ 0.0 };
			double GainThetadot1{ 0.0 };
			double GainThetadot2{ 0.0 };
			double GainDeCom{ 0.0 };
			double GainDeIntegral{ 0.0 };
			double SaturationDeIntegral{ 0.0 };
		}pitchCAS;

		struct {
			double TASErrorLimit{ 0.0 };
			double TASErrorGain{ 0.0 };
			double Kp{ 0.0 };
			double Ki{ 0.0 };
			double Kd{ 0.0 };
		}autothrottle;

		struct {

		}rollCAS;

		struct {

		}yawCAS;

	};



	struct modelparameter {

		struct {
			Matrix<double, Eigen::Dynamic, 3> Chart; // an N by 3 matrix , first col: J, second col: CT, third col: CP
			double diameter{ 0.0 };
			double minimumAngularRate{0.0};
			double shaftinertia{ 0.0 };
		}propeller;

		struct {
			Eigen::Matrix<double, Dynamic, 2> TorqueRPMChart;                      // output power versus RPM chart  an N by 2 matrix 0 RPM 1 Torque (m*s)
			Eigen::Matrix<double, Dynamic, 2> PowerMixtureChart;                   // power factor versus mixture chart an N by 2 matrix  0 RPM  1 toque factor
			Eigen::Matrix<double, Dynamic, 2> MixturePowerFactorSFCfactorChart;    // sfc factor versus mixture chart   an N by 2 matrix 0 RPM  1 SFC factor
			double idle_RPM{ 0.0 };                  // PRM 
			double shaft_damping{ 0.0 };             // shaft damping below the idle RPM
			double sfc{ 0.0 };						  // specific fuel consumption LB/(BHP*HR) pounds of fuel per break horse power per hour
			double superchargerfactor{ 0.0 };        // the power factor by turnning on super charger
			double krho0{ 0.0 };                     // rate parameter for manifold density amplifier 
			double krho1{ 0.0 };					  // base parameter for manifold density amplifier  
			double stater_max_torque{ 0.0 };         // the maximum torque the starter could provide 
			double stater_breakaway_RPM{ 0.0 };      // the PRM where the starter torque begins to decline with RPM
			double stater_zero_torque_RPM{ 0.0 };    // if below this torque, the engine produces no torque output
		}pistonengine;

		struct {
			double EmptyWeight{ 0.0 }; // the empty weight of the aircraft, excluding payload, passenger and fuel
			double Pilot1{ 0.0 };
			double Pilot2{ 0.0 };
			double Pilot3{ 0.0 };
			double Pilot4{ 0.0 };
			Matrix3d J; // moment of inertia
		}inertia;

		struct {
			double Span{ 0.0 };
			double MeanChord{ 0.0 };
			double ReferenceArea = 16.1651289600000;
		}geometry;

		struct {
			struct {
				gearparameter param;
				double MaxSteering;
			}nosegear;
			struct {
				gearparameter param;
			}leftgear;
			struct {
				gearparameter param;
			}rightgear;
		}gear;


		// autopilot parameters
		aerodynamicsparameter aerodynamics;
		autopilotparameter autopilot;
		// solver configuration
		simulationcontrol::SolverConfig Config;

	};

	struct modellist { // list containing all the model indexes of the system
		struct {
			unsigned int planekinematics{ 0 }; // TO DO: potential risk of forgetting definintion of system
			unsigned int planedynamics{ 0 };
			unsigned int gravityinertial{ 0 };
			//unsigned int gravity
			unsigned int gravitybody{ 0 };
			unsigned int Vbdot;
			unsigned int crossproduct{ 0 };
			unsigned int product{ 0 };
			unsigned int height{ 0 };
			unsigned int climbrate{ 0 };
			unsigned int sumtotalinertialforce{ 0 };
			unsigned int ACCxfilter{ 0 };
			unsigned int ACCyfilter{ 0 };
			unsigned int ACCzfilter{ 0 };
			unsigned int sumtotalbodyforce{ 0 };
			unsigned int rotation2inertialframe{ 0 };
			unsigned int planecurrentweight{ 0 };
			unsigned int loadfactor{ 0 };
			unsigned int loadfactorfluy{ 0 };
			unsigned int loadfactorfluz{ 0 };
			unsigned int sumtotalbodymoment{ 0 };
		}dynamics;

		struct {
			unsigned int aeroforcemoment{ 0 };
			unsigned int aeroangle{ 0 };
			unsigned int atmosphere{ 0 };// use the IAS for now, will change this later
			unsigned int filteredAOArate{ 0 };
			unsigned int filteredBetarate{ 0 };
		}aerodynamics;

		struct {
			unsigned int propeller{ 0 };
			unsigned int pistonengine{ 0 };
			unsigned int shaftdynamics{ 0 };
			unsigned int shaftinertia{ 0 };
			unsigned int omega2rps{ 0 };
			unsigned int totaltorque{ 0 };
			unsigned int propellerorientation{ 0 };
			unsigned int thrustforce{ 0 };
		}engine;

		struct {
			unsigned int GainThetadot1{ 0 };
			unsigned int GainThetadot2{ 0 };
			unsigned int GainDeCom{ 0 };
			unsigned int Deintegral{ 0 };
			unsigned int SaturationDeIntegral{ 0 };
			unsigned int GainDeIntegral{ 0 };
			unsigned int SumDeCom{ 0 };
			unsigned int SumDeltaDz{ 0 };
			unsigned int SumCstar1{ 0 };
			unsigned int ProductGammaPhi{ 0 };
			unsigned int CosGammaRoll{ 0 }; // 0 for gamma, 1 for roll
			unsigned int SaturationCosPhi{ 0 };
			unsigned int GainVS{ 0 };
			unsigned int GainAltitError{ 0 };
			unsigned int SaturationAltit{ 0 };
			unsigned int SumAltitError{ 0 };
			unsigned int SumVSError{ 0 };
			unsigned int PitchCASSwitch{ 0 };
			unsigned int PitchIntegralSwitch{ 0 };
			unsigned int PitchCASDeInputSwitch{ 0 };
		}pitchCAS;

		struct
		{
			unsigned int SumTASError{ 0 };
			unsigned int SaturateTASError{ 0 };
			unsigned int GainTASError{ 0 };
			unsigned int PIDThrottleCom{ 0 };
			unsigned int SumTotoalThrottle{ 0 };
			unsigned int AutoThrottleSwitch{ 0 };
		}autothrottle;

		struct {
			unsigned int NoseGearNormalForce{ 0 };
			unsigned int LeftGearNormalForce{ 0 };
			unsigned int RightGearNormalForce{ 0 };
			unsigned int NoseGearFrictionForce{ 0 };
			unsigned int LeftGearFrictionForce{ 0 };
			unsigned int RightGearFrictionForce{ 0 };
			unsigned int TotalGearMoment{ 0 };
			unsigned int TotalGearForce{ 0 };
			unsigned int GearMomentToBody{ 0 };
			unsigned int GroundNormal{ 0 };
		}landinggear;

		// some temporary blocks
		struct {
			unsigned int fixedthrottle;
			unsigned int fixedmixture;
			unsigned int fixedfuelstate;
		}temp;

	};

	struct initialcondition {
		struct {
			double roll{ 0.0 };
			double pitch{ 0.0 };
			double yaw{ 0.0 };
			double omegax{0.0};
			double omegay{ 0.0 };
			double omegaz{ 0.0 };
			double inertialvelocityx{ 0.0 };
			double inertialvelocityy{ 0.0 };
			double inertialvelocityz{ 0.0 };
			double inertialpositionx{ 0.0 };
			double inertialpositiony{ 0.0 };
			double inertialpositionz{ 0.0 };
		}plane;

		struct {
			double propellerRPM;
		}engine;
	};

	struct GNCdata {
		double TAS;
	};

	// wrapper to estabilish the C172 model using the solver components
	class AircraftDynamicModel {
	public:
		// the constructor assumbles the simualtion parts based on parameters and initial conditions. 
		AircraftDynamicModel(const modelparameter& param, const  initialcondition& IC);
		~AircraftDynamicModel();
		// run the simulation by executing this function
		void UpdateSimulation(const C172input& input); 
		// run this function when you wish to terminate the simulation, 
		// the function will store the logged data and close the logger file
		void EndSimulation();
		// reset the parameters if the isRunning is false. Return ture if reset successful
		bool ResetParameter(const modelparameter& param);
		// reset the simulation and initial condition if the isRunning is false. Return ture if reset successful
		bool ResetSimulation(const  initialcondition& IC);
		// display parameters
		void DisplayParameters();
		// display initial conditions
		void DisplayInitialConditions();
		// get aerodynamic block output
		void DisplayAerodynamicInfo();
		// get engine info
		void DisplayEngineInfo();
		// 
		void DisplayPropeller();
		// 
		void DisplayGearInfo();
		// 
		void DisplayGNCInfo();
		const GNCdata* GetGNCInfo();
	private:
		// a flag to show whether the simulation is running. It is set to false initially.
		// If UpdateSimulation is executed, then it is set to true.
		// If the EndSimulation is executed, then it is set to false.
		bool isRunning{ false };
		bool modelok{ false };
		modellist Modelist;
		VectorXd extern_input;  // the external input
		// backuped parameters and initial conditions
		modelparameter parameter;
		initialcondition currentIC;
		simulationcontrol::SimController SimInstance1;
		// input mapping index
		unsigned int speedcommandindex{ 0 };
		unsigned int trimthrottle{ 0 };
		unsigned int altcommand{ 0 };
		unsigned int autothrottlePIDenable{ 0 };
		unsigned int steering{ 0 };
		unsigned int nosegearbreak{ 0 };
		unsigned int leftgearbreak{ 0 };
		unsigned int rightgearbreak{ 0 };
		unsigned int nosegearswitch{ 0 };
		unsigned int leftgearswitch{ 0 };
		unsigned int rightgeaswitch{ 0 };
		unsigned int pitchCASMasterSwitch{ 0 };
		unsigned int pitchCASIntegralSwitch{ 0 };
		unsigned int altitudeHoldSwitch{ 0 };
		unsigned int autothrottleSwitch{ 0 };
		unsigned int throttleinput{ 0 };
		unsigned int elevatorinput{ 0 };
		// flight data
		GNCdata gncdata;

		// establish the aircraft model
		// step 1. define the rigid body
		void DefineRigidbody();
		// step 2. define the aerodynamics
		void DefineAerodynamics();
		// step 3. define the engine
		void DefineEngine();
		// step 4. define the autopilot
		void DefineAutopilot();
		// step 5. define landing gear
		void DefineLandingGear();
		// step 6. define the logging
		void DefineLogging();
		// connect system
		void ConnectSystems();
		// 
	};
}