/*
___________________________________________________________________________________________________
Author: Longhao Qian
Data:   2020 11 01

the wrapper class for the fixed-wing aircraft simulation 
___________________________________________________________________________________________________
*/
#pragma once
#include "SimController.h"
namespace aircraft {
	enum stickinput {
		INPUT_AILERON = 0,
		INPUT_ELEVATOR,
		INPUT_RUDDER,
		INPUT_FLAP,
	};

	enum engineinput {
		INPUT_STARTER = 0,
		INPUT_THROTTLE,
		INPUT_MIXTURE,
	};

	enum autopliotpitchmode {
		PITCHMODE_NONE = 0,  // the pitch CAS is off
		PITCHMODE_CAS,       // the pitch CAS is on (C star controller). Normal Acc Command Mode
		PITCHMODE_GAMMA,     // flight path hold mode
		PITCHMODE_ALTITUDE,  // altitude command mode
	};

	enum autopliotrollhmode {
		ROLLMODE_NONE = 0, // the roll CAS is off
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
			bool autopilotmaster;   // master swith of the entire autopilot system
			struct {
				autopliotpitchmode mode{ PITCHMODE_NONE };
				double commandaltitude;
				double commandgamma;
			}pitchCAS;

			struct {

			}rollCAS;

			struct {
				bool ON;
				double targetspeed; // target true airspeed (m/s)
			}autothrottle;

		}autopilot;
	};

	struct aerodynamics {
		struct {
			double CLalpha{0.0};
			double CLalpha_squared{ 0.0 };
			double CLalpha_cubed{ 0.0 };
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

	struct autopilot {
		
	};

	struct planeconfig {
		simulationcontrol::SolverConfig config;
		struct {

		};
	};

	struct modelparameter {
		struct {
			double WingSpan;
			double MeanChord;
			double ReferenceArea;
		}aerogeometric;

		struct {
			Eigen::Matrix<double, Dynamic, 2> TorqueRPMChart;                         // output power versus RPM chart  an N by 2 matrix 0 RPM 1 Torque (m*s)
			Eigen::Matrix<double, Dynamic, 2> PowerMixtureChart;                     // power factor versus mixture chart an N by 2 matrix  0 RPM  1 toque factor
			Eigen::Matrix<double, Dynamic, 2> MixturePowerFactorSFCfactorChart;    // sfc factor versus mixture chart   an N by 2 matrix 0 RPM  1 SFC factor
		}propeller;

		struct {

		}pistonengine;

		double MinAirspeed;


	};

	struct initialcondition {

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
	private:
		// a flag to show whether the simulation is running. It is set to false initially.
		// If UpdateSimulation is executed, then it is set to true.
		// If the EndSimulation is executed, then it is set to false.
		bool isRunning; 
		// backuped parameters and initial conditions
		modelparameter param;
		initialcondition currentIC;
	};
}