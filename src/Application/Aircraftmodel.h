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

	struct C172input {
		struct {
			bool starter;
			double throttle;
			double mixture;
		}engine;
		struct {
			double elevator;
			double aileron;
			double rudder;
			double elevatortrim;
		}controlsurface;
		struct {
			bool autopilotmaster;
			struct {
				bool ON;
				double targetspeed; // target true airspeed (m/s)
			}autothrottle;
			struct {
				bool ON;
				double altcommand; // command altitude (m)
			}altitudehold;
		}autopilot;
	};

	struct aeroparameter {
		struct {
			double CLalpha;
		}lift;
		struct {

		}side;
		struct {

		}drag;

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