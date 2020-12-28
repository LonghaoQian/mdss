// C172test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
#include "Aircraftmodel.h"
#include <time.h>

//#define DEBUG

enum FlightInitialCondition {
	INIT_MODE_GROUND = 0,
	INIT_MODE_CRUISE,
};

int main()
{
	
#ifndef DEBUG

	/*------------- define initial condition --------------------*/
	aircraft::initialcondition C172initialcondition;

	FlightInitialCondition initmode = INIT_MODE_GROUND;
	FlightInitialCondition flightstate = INIT_MODE_GROUND;
	switch (initmode) {
		case INIT_MODE_GROUND: {
			C172initialcondition.engine.propellerRPM = 2200.0; // RPM
			C172initialcondition.plane.omegax = 0.0;
			C172initialcondition.plane.omegay = 0.0;
			C172initialcondition.plane.omegaz = 0.0;
			C172initialcondition.plane.inertialpositionx = 0.0;
			C172initialcondition.plane.inertialpositiony = 0.0;
			C172initialcondition.plane.inertialpositionz = -2.0;// initial height NED frame
			C172initialcondition.plane.inertialvelocityx = 0.0; // 
			C172initialcondition.plane.inertialvelocityy = 0.0;
			C172initialcondition.plane.inertialvelocityz = 0.0;
			C172initialcondition.plane.roll = 0.0;
			C172initialcondition.plane.pitch = 0.0 / 57.3;
			C172initialcondition.plane.yaw = 0.0 / 57.3;

			break;
		}
		case INIT_MODE_CRUISE: {
			C172initialcondition.engine.propellerRPM = 2200.0; // RPM
			C172initialcondition.plane.omegax = 0.0;
			C172initialcondition.plane.omegay = 0.0;
			C172initialcondition.plane.omegaz = 0.0;
			C172initialcondition.plane.inertialpositionx = 0.0;
			C172initialcondition.plane.inertialpositiony = 0.0;
			C172initialcondition.plane.inertialpositionz = -500.0;// initial height NED frame
			C172initialcondition.plane.inertialvelocityx = 40.0; // 
			C172initialcondition.plane.inertialvelocityy = 0.0;
			C172initialcondition.plane.inertialvelocityz = 0.0;
			C172initialcondition.plane.roll = 0.0;
			C172initialcondition.plane.pitch = 1.0 / 57.3;
			C172initialcondition.plane.yaw = 0.0 / 57.3;

			break;
		}
		default: {// default case is te ground
			C172initialcondition.engine.propellerRPM = 2200.0; // RPM
			C172initialcondition.plane.omegax = 0.0;
			C172initialcondition.plane.omegay = 0.0;
			C172initialcondition.plane.omegaz = 0.0;
			C172initialcondition.plane.inertialpositionx = 0.0;
			C172initialcondition.plane.inertialpositiony = 0.0;
			C172initialcondition.plane.inertialpositionz = -2.0;// initial height NED frame
			C172initialcondition.plane.inertialvelocityx = 0.0; // 
			C172initialcondition.plane.inertialvelocityy = 0.0;
			C172initialcondition.plane.inertialvelocityz = 0.0;
			C172initialcondition.plane.roll = 0.0;
			C172initialcondition.plane.pitch = 0.0 / 57.3;
			C172initialcondition.plane.yaw = 0.0 / 57.3;
			break;
		}
	}

	/*----------------define system parameter ---------------------*/
	simulationcontrol::SolverConfig config1;
	config1.eposilon = 0.00001;
	config1.adaptive_step = false;
	config1.frame_step = 0.02;
	config1.mim_step = 0.005;
	config1.start_time = 0.0;
	config1.solver_type = RungeKuttaFamily::RUNGKUTTA45;
	config1.loggingconfig.filename = "c172log.txt";
	config1.loggingconfig.uselogging = true;
	config1.loglevel = simulationcontrol::LOGLEVEL_ERROR;


	aircraft::modelparameter C172parameter;
	C172parameter.Config = config1;
	C172parameter.geometry.MeanChord    = 1.493520000000000; // m
	C172parameter.geometry.ReferenceArea = 16.1651289600000; // m^2
	C172parameter.geometry.Span = 10.097; // m

	C172parameter.aerodynamics.MinAirspeed = 0.1; // m/s
	C172parameter.aerodynamics.lift.CL0 = 0.27;
	C172parameter.aerodynamics.lift.CLalpha = 4.6564;
	C172parameter.aerodynamics.lift.CLalphaflap = 0.0;
	C172parameter.aerodynamics.lift.CLalpha_cubed = 0.0;
	C172parameter.aerodynamics.lift.CLalpha_dot = 1.7;
	C172parameter.aerodynamics.lift.CLalpha_squared = 0.0;
	C172parameter.aerodynamics.lift.CLde = -0.24;
	C172parameter.aerodynamics.lift.CLflap = 0.0;
	C172parameter.aerodynamics.lift.CLflap_squared = 0.0;
	C172parameter.aerodynamics.lift.CLq = 3.9;

	C172parameter.aerodynamics.drag.CD0_ = 0.032;
	C172parameter.aerodynamics.drag.CDbeta_ = 0.17;
	C172parameter.aerodynamics.drag.CDde_ = 0.06;
	C172parameter.aerodynamics.drag.CDground_ = 0.0;
	C172parameter.aerodynamics.drag.CD_alpha_ = 0.49;
	C172parameter.aerodynamics.drag.CD_alpha_squared_ = 0.0;
	C172parameter.aerodynamics.drag.CD_flap_ = 0.0;
	C172parameter.aerodynamics.drag.CD_flap_squared_ = 0.0;

	C172parameter.aerodynamics.side.CYb = -0.3095;
	C172parameter.aerodynamics.side.CYda = -0.05; 
	C172parameter.aerodynamics.side.CYdr = 0.098;
	C172parameter.aerodynamics.side.CYp = -0.037;
	C172parameter.aerodynamics.side.CYr = 0.21;

	C172parameter.aerodynamics.roll.Clb_ = -0.0891;
	C172parameter.aerodynamics.roll.Clda_ = 0.23;
	C172parameter.aerodynamics.roll.Cldr_ = 0.0147;
	C172parameter.aerodynamics.roll.Clr_ = 0.0147;
	C172parameter.aerodynamics.roll.Clp_ = -0.47;

	C172parameter.aerodynamics.pitch.Cm0_ = 0.1;
	C172parameter.aerodynamics.pitch.Cmadot_ = -5.2;
	C172parameter.aerodynamics.pitch.Cmalpha_ = -1.8;
	C172parameter.aerodynamics.pitch.Cmde_ = -1.18;
	C172parameter.aerodynamics.pitch.Cmq_ = -12.4;
	C172parameter.aerodynamics.pitch.Cm_flap_ = 0.0;
	C172parameter.aerodynamics.pitch.Cm_flap_squared_ = 0.0;

	C172parameter.aerodynamics.yaw.Cnb_ = 0.0650;
	C172parameter.aerodynamics.yaw.Cnda_ = 0.0053;
	C172parameter.aerodynamics.yaw.Cndr_ = -0.043;
	C172parameter.aerodynamics.yaw.Cnp_ = -0.03;
	C172parameter.aerodynamics.yaw.Cnr_ = -0.099;
	
	C172parameter.inertia.EmptyWeight = 1454.0*0.453592; //kg
	C172parameter.inertia.Pilot1 = 190 * 0.453592; //kg
	C172parameter.inertia.Pilot2 = 190 * 0.453592; //kg
	C172parameter.inertia.Pilot3 = 0.0;
	C172parameter.inertia.Pilot4 = 0.0;
	C172parameter.inertia.J.setZero();
	C172parameter.inertia.J(0, 0) = 1285.31541660000;
	C172parameter.inertia.J(1, 1) = 1824.93096070000;
	C172parameter.inertia.J(2, 2) = 2666.89390765000;

	C172parameter.pistonengine.idle_RPM = 550;
	C172parameter.pistonengine.krho0 = 1.115;
	C172parameter.pistonengine.krho1 = -0.1146;
	C172parameter.pistonengine.sfc = 0.435*(0.453592 / (745.7 * 3600)); // LB / BHP / HR
	C172parameter.pistonengine.MixturePowerFactorSFCfactorChart.resize(11, 2);
	C172parameter.pistonengine.MixturePowerFactorSFCfactorChart<< -1.0000, 0.8500,
		-0.8000, 0.8000,
		-0.6000, 0.8500,
		-0.4000, 0.9000,
		-0.2000, 0.9500,
		0.00, 1.0000,
		0.2000, 1.0600,
		0.4000, 1.1200,
		0.6000, 1.1800,
		0.8000, 1.2400,
		1.0000, 1.3000;
	C172parameter.pistonengine.PowerMixtureChart.resize(11, 2);
	C172parameter.pistonengine.PowerMixtureChart<< -1.0000, 0.0500,
		-0.8000, 0.8000,
		-0.6000, 0.9600,
		-0.4000, 0.9800,
		-0.2000, 0.9990,
		0.00, 1.0000,
		0.2000, 0.9990,
		0.4000, 0.9900,
		0.6000, 0.9700,
		0.8000, 0.9500,
		1.0000, 0.9300;
	C172parameter.pistonengine.TorqueRPMChart.resize(24, 2);
	C172parameter.pistonengine.TorqueRPMChart << 500, 725.763194472288, // Nm vs RPM
		600, 664.410616734446,
		700, 620.587346921702,
		800, 587.719894562144,
		900, 562.156320504710,
		1000, 541.705461258763,
		1100, 524.972940057533,
		1200, 511.029172389842,
		1300, 499.230599747950,
		1400, 489.117537483470,
		1500, 480.352883520921,
		1600, 472.683811303691,
		1700, 465.916982876723,
		1800, 459.902024274974,
		1900, 454.520219210251,
		2000, 449.676594652001,
		2100, 445.294267670726,
		2200, 440.913828809650,
		2300, 433.880170608433,
		2400, 430.340355696709,
		2500, 424.292329076107,
		2600, 418.709535272474,
		2700, 410.955654989650,
		2800, 406.247941960793;
	C172parameter.pistonengine.shaft_damping = -0.005;
	C172parameter.pistonengine.superchargerfactor = 1.0;
	C172parameter.pistonengine.stater_breakaway_RPM = 560;
	C172parameter.pistonengine.stater_zero_torque_RPM = 700;

	C172parameter.propeller.Chart.resize(16, 3);
	C172parameter.propeller.Chart << 0.0, 0.0990, 0.0400,
		0.1000, 0.0950, 0.0406,
		0.2000, 0.0880, 0.0406,
		0.3000, 0.0780, 0.0400,
		0.4000, 0.0645, 0.0366,
		0.5000, 0.0495, 0.0318,
		0.6000, 0.0340, 0.0250,
		0.7000, 0.0185, 0.0160,
		0.8000, 0.0040, 0.0050,
		0.9000, -0.0160, -0.0067,
		1.0000, -0.0300, -0.0150,
		1.1000, -0.0400, -0.0200,
		1.2000, -0.0500, -0.0250,
		1.5000, -0.0550, -0.0270,
		1.6000, -0.0650, -0.0300,
		2.0000, -0.0750, -0.0330;

	C172parameter.propeller.diameter = 76 * 0.0254; //m
	C172parameter.propeller.minimumAngularRate = 1.0;
	C172parameter.propeller.shaftinertia = 1.6700/(2.0*M_PI);
	// landing gear parameter
	C172parameter.gear.nosegear.MaxSteering = 10.0;
	C172parameter.gear.nosegear.param.CompressDamping = 500.0 * 4.4482 / 0.3048;
	C172parameter.gear.nosegear.param.Hmax = 10.0;
	C172parameter.gear.nosegear.param.NeMin = 0.1;
	C172parameter.gear.nosegear.param.Nex = 0.0;
	C172parameter.gear.nosegear.param.Ney = 0.0;
	C172parameter.gear.nosegear.param.Nez = 1.0;
	C172parameter.gear.nosegear.param.ReboundDamping = 800.0 * 4.4482 / 0.3048;
	C172parameter.gear.nosegear.param.Rex = 1.2141;
	C172parameter.gear.nosegear.param.Rey = 0.0;
	C172parameter.gear.nosegear.param.Rez = 1.4351;
	C172parameter.gear.nosegear.param.Sigma0 = 3.0;
	C172parameter.gear.nosegear.param.Stiffness = 1800.0 * 4.4482 / 0.3048;
	C172parameter.gear.nosegear.param.SigmaDynamic = 0.4;
	C172parameter.gear.nosegear.param.SigmaRoll = 0.15;
	C172parameter.gear.nosegear.param.SigmaStatic = 0.5;
	C172parameter.gear.nosegear.param.Vlimit = 5.0;
	C172parameter.gear.nosegear.param.VrelaxationRoll = 0.3;
	C172parameter.gear.nosegear.param.VrelaxationSide = 0.3;

	C172parameter.gear.leftgear.param.CompressDamping = 160.0 * 4.4482 / 0.3048;
	C172parameter.gear.leftgear.param.ReboundDamping = 320.0 * 4.4482 / 0.3048;
	C172parameter.gear.leftgear.param.Hmax = 10.0;
	C172parameter.gear.leftgear.param.NeMin = 0.1;
	C172parameter.gear.leftgear.param.Nex = 0.0;
	C172parameter.gear.leftgear.param.Ney = 0.0;
	C172parameter.gear.leftgear.param.Nez = 1.0;
	C172parameter.gear.leftgear.param.Rex = -0.4369;
	C172parameter.gear.leftgear.param.Rey = -1.2763;
	C172parameter.gear.leftgear.param.Rez = 1.3960;
	C172parameter.gear.leftgear.param.Stiffness = 5400.0 * 4.4482 / 0.3048;
	C172parameter.gear.leftgear.param.Sigma0 = 3.0;
	C172parameter.gear.leftgear.param.SigmaDynamic = 0.4;
	C172parameter.gear.leftgear.param.SigmaRoll = 0.15;
	C172parameter.gear.leftgear.param.SigmaStatic = 0.5;
	C172parameter.gear.leftgear.param.Vlimit = 5.0;
	C172parameter.gear.leftgear.param.VrelaxationRoll = 0.3;
	C172parameter.gear.leftgear.param.VrelaxationSide = 0.3;

	C172parameter.gear.rightgear.param.CompressDamping = 160 * 4.4482 / 0.3048;
	C172parameter.gear.rightgear.param.ReboundDamping  = 320.0 * 4.4482 / 0.3048;
	C172parameter.gear.rightgear.param.Hmax = 10.0;
	C172parameter.gear.rightgear.param.NeMin = 0.1;
	C172parameter.gear.rightgear.param.Nex = 0.0;
	C172parameter.gear.rightgear.param.Ney = 0.0;
	C172parameter.gear.rightgear.param.Nez = 1.0;
	C172parameter.gear.rightgear.param.Rex = -0.4369;
	C172parameter.gear.rightgear.param.Rey = 1.2763;
	C172parameter.gear.rightgear.param.Rez = 1.3960;;
	C172parameter.gear.rightgear.param.Sigma0 = 3.0;
	C172parameter.gear.rightgear.param.SigmaDynamic = 0.4;
	C172parameter.gear.rightgear.param.SigmaRoll = 0.15;
	C172parameter.gear.rightgear.param.SigmaStatic = 0.5;
	C172parameter.gear.rightgear.param.Vlimit = 5.0;
	C172parameter.gear.rightgear.param.Stiffness = 5400.0 * 4.4482 / 0.3048;
	C172parameter.gear.rightgear.param.VrelaxationRoll = 0.3;
	C172parameter.gear.rightgear.param.VrelaxationSide = 0.3;

	// auto throttle parameters
	C172parameter.autopilot.autothrottle.TASErrorLimit = 10.0;
	C172parameter.autopilot.autothrottle.TASErrorGain = 0.1;
	C172parameter.autopilot.autothrottle.Kd = 0.0;
	C172parameter.autopilot.autothrottle.Ki = 0.04;
	C172parameter.autopilot.autothrottle.Kp = 15.0;
	// altitude command parameters
	C172parameter.autopilot.pitchCAS.AltitudeErrorLimit = 10.0;
	C172parameter.autopilot.pitchCAS.AltitudeErrorToVSGain = 0.3556;
	C172parameter.autopilot.pitchCAS.VSGain = -1.5 / 9.0;
	C172parameter.autopilot.pitchCAS.GainDeCom = -2.0;
	C172parameter.autopilot.pitchCAS.GainThetadot1 = 1.0;
	C172parameter.autopilot.pitchCAS.GainThetadot2 = 5.612;
	C172parameter.autopilot.pitchCAS.GainDeIntegral = 0.4;
	C172parameter.autopilot.pitchCAS.SaturationDeIntegral = 0.4;
	// heading command


	//actuators:

	C172parameter.actuator.aileron.Ts = 0.05;
	C172parameter.actuator.elevator.Ts = 0.05;
	C172parameter.actuator.rudder.Ts = 0.05;
	C172parameter.actuator.flap.MaxSpeed = 20;
	C172parameter.actuator.flap.Shapfactor = 10;
	/*------------ define control inputs ----------------------*/
	aircraft::C172input controlinput;



	aircraft::AircraftDynamicModel C172aicraftmodel(C172parameter, C172initialcondition);
	C172aicraftmodel.DisplayAerodynamicInfo();
	C172aicraftmodel.DisplayActuatorInfo();
	clock_t t; // measure the time used
	int start = 0;
	std::cin >> start;
	if (start != 1) {
		std::cout << " Abort... " << std::endl;
		return 0;
	}
	int N_steps = 3000;
	t = clock();
	for (int i = 0; i < N_steps; i++) {
		// run the control logic
		switch (flightstate)
		{
		case INIT_MODE_GROUND:
			// on the ground 
			controlinput.controlsurface.elevator = 0.0;
			controlinput.engine.throttle = 0.0;
			controlinput.autopilot.autopilotmaster = false;
			controlinput.autopilot.pitchCAS.commandaltitude = 30.0;//m
			// define auto throttle input
			controlinput.autopilot.autothrottle.ON = true;
			controlinput.autopilot.autothrottle.targetspeed = 40.0;// m / s
			controlinput.autopilot.autothrottle.trimthrottle = 0.7;
			// 

			// gear command
			controlinput.gear.gearbreak = false;
			controlinput.gear.geardown = true;
			controlinput.gear.steering = 0.0;
			break;
		case INIT_MODE_CRUISE:
			controlinput.controlsurface.elevator = 0.0;
			controlinput.engine.throttle = 0.0;
			controlinput.autopilot.autopilotmaster = true;
			controlinput.autopilot.pitchCAS.commandaltitude = 100.0;//m
			// define auto throttle input
			controlinput.autopilot.autothrottle.ON = true;
			controlinput.autopilot.autothrottle.targetspeed = 55.0;// m / s
			controlinput.autopilot.autothrottle.trimthrottle = 0.7;
			// 

			// gear command
			controlinput.gear.gearbreak = false;
			controlinput.gear.geardown = true;
			controlinput.gear.steering = 0.0;
			break;
		default:

			break;
		}
		// update the aircraft state
		C172aicraftmodel.UpdateSimulation(controlinput);
		// determine mode
		if (C172aicraftmodel.GetGNCInfo()->TAS > 39.0) {
			flightstate = INIT_MODE_CRUISE;
		} else
		{
			flightstate = INIT_MODE_GROUND;
		}

	}
	t = clock() - t;
	std::cout << "Calculation Takes " << t << " clicks " << ((float)t) / CLOCKS_PER_SEC << " seconds \n";
	std::cout << "The speed  ratio is " << (double)N_steps * config1.frame_step / (((double)t) / CLOCKS_PER_SEC) << '\n';
	C172aicraftmodel.EndSimulation();
#endif

#ifdef DEBUG

	clock_t t;

	simulationcontrol::SolverConfig config1;
	config1.eposilon = 0.00001;
	config1.adaptive_step = false;
	config1.frame_step = 0.02;
	config1.mim_step = 0.005;
	config1.start_time = 0.0;
	config1.solver_type = RungeKuttaFamily::RUNGKUTTA45;
	config1.loggingconfig.filename = "datalog.txt";
	config1.loggingconfig.uselogging = true;
	config1.loglevel = simulationcontrol::LOGLEVEL_ERROR;
	simulationcontrol::SimController SimInstance1(config1);


	double Span  = 10.097;
	double MeanChord = 1.493520000000000;
	double MinAirspeed = 0.1;
	double ReferenceArea = 16.1651289600000;
	// plane mass
	double planemass = 1454.0*0.453592;
	// plane CG
	double CG_MAC = 0.25;
	// plane initial condition

	// define kinematics
	dynamics::RigidBodyKinematicsInitialCondition initial_condition;
	initial_condition.Euler0(mathauxiliary::EULER_ROLL) = 0.0;
	initial_condition.Euler0(mathauxiliary::EULER_PITCH) = 4.0/57.3;
	initial_condition.Euler0(mathauxiliary::EULER_YAW) = 0.0/57.3;
	initial_condition.Omega0(mathauxiliary::VECTOR_X) = 0.0;
	initial_condition.Omega0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.Omega0(mathauxiliary::VECTOR_Z) = 0.0;
	initial_condition.VI0(mathauxiliary::VECTOR_X) = 40.0;
	initial_condition.VI0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.VI0(mathauxiliary::VECTOR_Z) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_X) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_Z) = -7.0; // END frame
	unsigned int planekinematics = SimInstance1.AddSubSystem(initial_condition);
	// define dynamics
	dynamics::RigidBodyDynamicsParamter dynamics_parameter;
	dynamics_parameter.J.setZero();
	dynamics_parameter.J(0, 0) = 1285.31541660000; // kg m^2
	dynamics_parameter.J(1, 1) = 1824.93096070000;
	dynamics_parameter.J(2, 2) = 2666.89390765000;
	dynamics_parameter.m = planemass;
	unsigned int planedynamics = SimInstance1.AddSubSystem(dynamics_parameter);
	// define gravity 
	mathblocks::ConstantParameter gravity_param;
	gravity_param.value.resize(3, 1);
	gravity_param.value(0) = 0.0;
	gravity_param.value(1) = 0.0;
	gravity_param.value(2) = 9.81*planemass;
	unsigned int gravity = SimInstance1.AddSubSystem(gravity_param);

	mathblocks::MultiplicationParam multiple_1_param;
	multiple_1_param.Mode = mathblocks::MULTI_MATRIX;
	multiple_1_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
	multiple_1_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
	multiple_1_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
	multiple_1_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
	unsigned int product_1 = SimInstance1.AddSubSystem(multiple_1_param);

	mathblocks::MultiplicationParam multiple_2_param;
	multiple_2_param.Mode = mathblocks::MULTI_MATRIX;
	multiple_2_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
	multiple_2_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
	multiple_2_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
	multiple_2_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
	unsigned int product_2 = SimInstance1.AddSubSystem(multiple_2_param);

	mathblocks::CrossProductParameter cross_1_param;
	cross_1_param.mode = 0;
	unsigned int crossproduct1 = SimInstance1.AddSubSystem(cross_1_param);

	mathblocks::SumParameter sum_Vb_parameter;
	sum_Vb_parameter.input_dimensions = 3;
	sum_Vb_parameter.SignList.push_back(mathblocks::SUM_POSITIVE);
	sum_Vb_parameter.SignList.push_back(mathblocks::SUM_POSITIVE);
	unsigned int sum_Vb = SimInstance1.AddSubSystem(sum_Vb_parameter);

	// define aeroangle
	aero::AeroAngleParameter aeroangleparameter;
	aeroangleparameter.b_ = Span;
	aeroangleparameter.c_bar_ = MeanChord;
	aeroangleparameter.min_airspeed_ = MinAirspeed;
	unsigned int aeroanlge = SimInstance1.AddSubSystem(aeroangleparameter);

	// define aero force
	aero::AerosForceParameter aeroforceparam;

	aeroforceparam.b_ = Span;
	aeroforceparam.c_bar_ = MeanChord;
	aeroforceparam.S = ReferenceArea;

	aeroforceparam.AeroCoefficient.Lift.CL0_                = 0.377;
	aeroforceparam.AeroCoefficient.Lift.CLadot_             = 1.0;
	aeroforceparam.AeroCoefficient.Lift.CLde_               = 0.0;// -0.359366356655653;
	aeroforceparam.AeroCoefficient.Lift.CLq_ = 3.9;;// 3.9;
	aeroforceparam.AeroCoefficient.Lift.CL_alpha_ = 4.6564;// 4.6564;
	aeroforceparam.AeroCoefficient.Lift.CL_alpha_squared_ = -0.4;//-0.4;
	aeroforceparam.AeroCoefficient.Lift.CL_alpha_cubed_ = -0.1;;// -0.1;
	aeroforceparam.AeroCoefficient.Lift.CL_flap_ = 0.0;// 0.5;
	aeroforceparam.AeroCoefficient.Lift.CL_flap_squared_ = 0.0;// -0.2;

	aeroforceparam.AeroCoefficient.Drag.CD0_ = 0.032;
	aeroforceparam.AeroCoefficient.Drag.CDbeta_ =  0.17;
	aeroforceparam.AeroCoefficient.Drag.CDde_ = 0.06;
	aeroforceparam.AeroCoefficient.Drag.CD_flap_ = 0.021;
	aeroforceparam.AeroCoefficient.Drag.CD_flap_squared_ = - 0.005;
	aeroforceparam.AeroCoefficient.Drag.CDground_ = 0.0;
	aeroforceparam.AeroCoefficient.Drag.CD_alpha_ = 0.5;
	aeroforceparam.AeroCoefficient.Drag.CD_alpha_squared_ = 0.2;
	aeroforceparam.AeroCoefficient.Drag.CD_flap_ = 0.4;
	aeroforceparam.AeroCoefficient.Drag.CD_flap_squared_ = 0.3;

	aeroforceparam.AeroCoefficient.Side.CYb_ = -0.3095;
	aeroforceparam.AeroCoefficient.Side.CYda_ = -0.05;
	aeroforceparam.AeroCoefficient.Side.CYdr_ = 0.098;
	aeroforceparam.AeroCoefficient.Side.CYp_ = -0.0370;
	aeroforceparam.AeroCoefficient.Side.CYr_ = 0.21;	

	aeroforceparam.AeroCoefficient.Roll.Clb_ = -0.089100000000000;
	aeroforceparam.AeroCoefficient.Roll.Clda_ = 0.230000000000000;
	aeroforceparam.AeroCoefficient.Roll.Cldr_ = 0.014700000000000;
	aeroforceparam.AeroCoefficient.Roll.Clp_ = -0.470000000000000;
	aeroforceparam.AeroCoefficient.Roll.Clr_ = 0.08;

	aeroforceparam.AeroCoefficient.Pitch.Cm0_ = 0.120000000000000;
	aeroforceparam.AeroCoefficient.Pitch.Cmadot_ = -5.200000000000000;
	aeroforceparam.AeroCoefficient.Pitch.Cmalpha_ = -1.800000000000000;
	aeroforceparam.AeroCoefficient.Pitch.Cmde_ = -1.180000000000000;
	aeroforceparam.AeroCoefficient.Pitch.Cm_flap_ = -0.1;
	aeroforceparam.AeroCoefficient.Pitch.Cm_flap_squared_ = 0.05;
	aeroforceparam.AeroCoefficient.Pitch.Cmq_ = -12.4;

	aeroforceparam.AeroCoefficient.Yaw.Cnb_ = 0.065000000000000;
	aeroforceparam.AeroCoefficient.Yaw.Cnda_ = 0.005300000000000;
	aeroforceparam.AeroCoefficient.Yaw.Cndr_ = -0.043000000000000;
	aeroforceparam.AeroCoefficient.Yaw.Cnp_ = -0.030000000000000;
	aeroforceparam.AeroCoefficient.Yaw.Cnr_ = -0.099000000000000;

	unsigned int aeroforce = SimInstance1.AddSubSystem(aeroforceparam);

	//// force summation
	//mathblocks::SumParameter force_sum_param;
	//force_sum_param.input_dimensions = 3;
	//force_sum_param.num_of_inputs = 3; // areo gravity and thrust
	//unsigned int force_summation = SimInstance1.AddSubSystem(force_sum_param);
	//// moment summation
	mathblocks::ConstantParameter height_param;
	height_param.value.resize(1, 1);
	height_param.value(0) = 1300.0;
	unsigned int height = SimInstance1.AddSubSystem(height_param);

	geographic::StandardAtmosphereParameter atom_param;
	atom_param.atmoshpere_name_ = "atom_data";
	unsigned int atmoshpere = SimInstance1.AddSubSystem(atom_param);

	// force summation : aero, gravity, and thrust 
	source_sink::PeriodicWaveparameter signal_generator_param1;
	signal_generator_param1.amplitude = 0.3;
	signal_generator_param1.frequency = 0.5;
	signal_generator_param1.num_of_channels = 1;
	signal_generator_param1.phase_shift = 0.0;
	signal_generator_param1.waveshape = source_sink::SINE;
	unsigned int signal_generator1 = SimInstance1.AddSubSystem(signal_generator_param1);

	source_sink::PeriodicWaveparameter signal_generator_param2;
	signal_generator_param2.amplitude = 0.1;
	signal_generator_param2.frequency = 0.5;
	signal_generator_param2.num_of_channels = 1;
	signal_generator_param2.phase_shift = 0.0;
	signal_generator_param2.waveshape = source_sink::SINE;
	unsigned int signal_generator2 = SimInstance1.AddSubSystem(signal_generator_param2);

	source_sink::PeriodicWaveparameter signal_generator_param3;
	signal_generator_param3.amplitude = 0.4;
	signal_generator_param3.frequency = 0.5;
	signal_generator_param3.num_of_channels = 1;
	signal_generator_param3.phase_shift = 0.0;
	signal_generator_param3.waveshape = source_sink::SINE;
	unsigned int signal_generator3 = SimInstance1.AddSubSystem(signal_generator_param3);

	// propeller system
	propulsionsystem::PropellerChartFixedPitchParameter propeller_param;
	propeller_param.Chart.resize(16, 3);
	propeller_param.Chart << 0.0    ,0.0990   , 0.0400,
							0.1000  ,  0.0950 ,   0.0406,
							0.2000  ,  0.0880  ,  0.0406,
							0.3000  ,  0.0780  ,  0.0400,
		0.4000,    0.0645,    0.0366,
		0.5000,    0.0495,    0.0318,
		0.6000,    0.0340,    0.0250,
		0.7000,    0.0185,    0.0160,
		0.8000,    0.0040,    0.0050,
		0.9000, - 0.0160, - 0.0067,
		1.0000, - 0.0300, - 0.0150,
		1.1000, - 0.0400, - 0.0200,
		1.2000, - 0.0500, - 0.0250,
		1.5000, - 0.0550, - 0.0270,
		1.6000, - 0.0650, - 0.0300,
		2.0000, - 0.0750, - 0.0330;

	propeller_param.diameter = 76 * 0.0254; //m
	propeller_param.minimumAngularRate = 1.0;

	unsigned int propeller_1 = SimInstance1.AddSubSystem(propeller_param);

	// piston engine system
	propulsionsystem::PistonEngineParameter pistonengine_param;
	pistonengine_param.idle_RPM = 550.0;
	pistonengine_param.krho0 = 1.115;
	pistonengine_param.krho1 = -0.1146;
	pistonengine_param.sfc = 0.435*(0.453592 / (745.7 * 3600)); // LB / BHP / HR
	pistonengine_param.PowerMixtureChart.resize(11,2);
	pistonengine_param.PowerMixtureChart << -1.0000, 0.0500,
		-0.8000, 0.8000,
		-0.6000, 0.9600,
		-0.4000, 0.9800,
		-0.2000, 0.9990,
		0.00, 1.0000,
		0.2000, 0.9990,
		0.4000, 0.9900,
		0.6000, 0.9700,
		0.8000, 0.9500,
		1.0000, 0.9300;
	pistonengine_param.TorqueRPMChart.resize(24, 2);
	pistonengine_param.TorqueRPMChart << 500,	725.763194472288, // Nm vs RPM
		600,	664.410616734446,
		700,	620.587346921702,
		800,	587.719894562144,
		900,	562.156320504710,
		1000,	541.705461258763,
		1100,	524.972940057533,
		1200,	511.029172389842,
		1300,	499.230599747950,
		1400,	489.117537483470,
		1500,	480.352883520921,
		1600,	472.683811303691,
		1700,	465.916982876723,
		1800,	459.902024274974,
		1900,	454.520219210251,
		2000,	449.676594652001,
		2100,	445.294267670726,
		2200,	440.913828809650,
		2300,	433.880170608433,
		2400,	430.340355696709,
		2500,	424.292329076107,
		2600,	418.709535272474,
		2700,	410.955654989650,
		2800,	406.247941960793;
	pistonengine_param.MixturePowerFactorSFCfactorChart.resize(11, 2);
	pistonengine_param.MixturePowerFactorSFCfactorChart << -1.0000 ,   0.8500,
		- 0.8000,    0.8000,
		- 0.6000,    0.8500,
		- 0.4000 ,   0.9000,
		- 0.2000 ,   0.9500,
		0.00,    1.0000,
		0.2000,   1.0600,
		0.4000,   1.1200,
		0.6000,    1.1800,
		0.8000,    1.2400,
		1.0000,    1.3000;
	pistonengine_param.shaft_damping = -0.005;
	pistonengine_param.superchargerfactor = 1.0;
	pistonengine_param.stater_breakaway_RPM = 560;
	pistonengine_param.stater_zero_torque_RPM = 1000;
	


	unsigned int piston_engine_1 = SimInstance1.AddSubSystem(pistonengine_param);
	// shaft dynamics

	mathblocks::GainParameter shaft_inertia_param;
	shaft_inertia_param.K.resize(1, 1);
	shaft_inertia_param.K(0,0) = 1.0/ 1.6700;
	shaft_inertia_param.Mode = mathblocks::ElementWise;
	shaft_inertia_param.num_of_inputs = 1;

	unsigned int shaft_inertia = SimInstance1.AddSubSystem(shaft_inertia_param);

	mathblocks::GainParameter omega_rps_param;
	omega_rps_param.K.resize(1, 1);
	omega_rps_param.K(0, 0) = 1.0 / (2.0*M_PI);
	omega_rps_param.Mode = mathblocks::ElementWise;
	omega_rps_param.num_of_inputs = 1;

	unsigned int omega2rps = SimInstance1.AddSubSystem(omega_rps_param);

	linearsystem::IntegratorParameter shaftdynamics_param;
	shaftdynamics_param.num_of_channels = 1;
	linearsystem::IntegratorInitialCondition shaftdynamics_IC;
	shaftdynamics_IC.X_0.resize(1);
	shaftdynamics_IC.X_0(0)  = 2.0 * M_PI * 2000.0 / 60.0;

	unsigned int shaftdynamics = SimInstance1.AddSubSystem(shaftdynamics_param, shaftdynamics_IC);

	mathblocks::SumParameter total_torque_to_shaft_param;

	total_torque_to_shaft_param.input_dimensions = 1;
	total_torque_to_shaft_param.SignList.push_back(mathblocks::SUM_POSITIVE);
	total_torque_to_shaft_param.SignList.push_back(mathblocks::SUM_NEGATIVE);


	unsigned int  total_torque_to_shaft = SimInstance1.AddSubSystem(total_torque_to_shaft_param);

	mathblocks::ConstantParameter horizontal_wind_speed_param;

	horizontal_wind_speed_param.value.resize(3);
	horizontal_wind_speed_param.value(0) = 2.5; //(m/s)
	horizontal_wind_speed_param.value(1) = 90 / 57.3; // rad
	horizontal_wind_speed_param.value(2) = -1.2; //(m/s)

	unsigned int  horizontal_wind = SimInstance1.AddSubSystem(horizontal_wind_speed_param);

	mathblocks::ConstantParameter fixeddensity_param;

	fixeddensity_param.value.resize(1);
	fixeddensity_param.value(0) = 1.0; // kg/m^3

	unsigned int  fixeddensity = SimInstance1.AddSubSystem(fixeddensity_param);


	mathblocks::ConstantParameter fixedrps_param;

	fixedrps_param.value.resize(1);
	fixedrps_param.value(0) = 2000.0/60.0; // 2000RPM/60

	unsigned int  fixedrps = SimInstance1.AddSubSystem(fixedrps_param);

	mathblocks::ConstantParameter fixedthrottle_param;

	fixedthrottle_param.value.resize(1);
	fixedthrottle_param.value(0) = 0.5;

	unsigned int  fixedthrottle = SimInstance1.AddSubSystem(fixedthrottle_param);

	mathblocks::ConstantParameter fixedmxiture_param;

	fixedmxiture_param.value.resize(1);
	fixedmxiture_param.value(0) = -0.2;

	unsigned int  fixedmxiture = SimInstance1.AddSubSystem(fixedmxiture_param);

	mathblocks::ConstantParameter fixefuelstate_param;

	fixefuelstate_param.value.resize(1);
	fixefuelstate_param.value(0) = 1.0;

	unsigned int  fixefuelstate = SimInstance1.AddSubSystem(fixefuelstate_param);

	mathblocks::DivisionParameter division_param;
	division_param.SignList.push_back(mathblocks::DIVISION_PRODUCT);
	division_param.SignList.push_back(mathblocks::DIVISION_DIVISION);

	unsigned int division = SimInstance1.AddSubSystem(division_param);

	discontinuoussystem::SaturationParameter saturation_param;
	saturation_param.lower_bound = 0.6;
	saturation_param.upper_bound = 1.0;
	saturation_param.num_of_channels = 1;
	saturation_param.type = discontinuoussystem::SATURATION_BOTH;

	unsigned int saturation = SimInstance1.AddSubSystem(saturation_param);

	mathblocks::TrigonometryParameter trig_param;
	trig_param.num_of_channels = 1;
	trig_param.type = mathblocks::COS;
	unsigned int  trig_cos1 = SimInstance1.AddSubSystem(trig_param);

	unsigned int  trig_cos2 = SimInstance1.AddSubSystem(trig_param);

	groundcontact::SimpleGearNormalForceParameter nose_gear_param;
	nose_gear_param.MinNz = 0.5;
	nose_gear_param.MaxHeight = 10.0;
	nose_gear_param.kCompress = 1800.0 * 4.4482 / 0.3048;
	nose_gear_param.dRebound = 2000.0 * 4.4482 / 0.3048;
	nose_gear_param.dCompress = 500.0 * 4.4482 / 0.3048;
	nose_gear_param.GearDirection(mathauxiliary::VECTOR_X) = 0.0;
	nose_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = 0.0;
	nose_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = 1.0;
	nose_gear_param.GearPosition(mathauxiliary::VECTOR_X) = 1.2141;
	nose_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = 0.0;
	nose_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = 1.4351;
	nose_gear_param.isSteering = true;
	unsigned int nose_gear = SimInstance1.AddSubSystem(nose_gear_param);

	groundcontact::SimpleGearNormalForceParameter left_gear_param;
	left_gear_param.MinNz = 0.5;
	left_gear_param.MaxHeight = 10.0;
	left_gear_param.kCompress = 5400.0 * 4.4482 / 0.3048;
	left_gear_param.dRebound = 320.0 * 4.4482 / 0.3048;
	left_gear_param.dCompress = 160.0 * 4.4482 / 0.3048;
	left_gear_param.GearDirection(mathauxiliary::VECTOR_X) = 0.0;
	left_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = 0.0;
	left_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = 1.0;
	left_gear_param.GearPosition(mathauxiliary::VECTOR_X) = -0.4369;
	left_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = -1.2763;
	left_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = 1.3960;
	left_gear_param.isSteering = false;
	unsigned int left_gear = SimInstance1.AddSubSystem(left_gear_param);

	groundcontact::SimpleGearNormalForceParameter right_gear_param;
	right_gear_param.MinNz = 0.5;
	right_gear_param.MaxHeight = 10.0;
	right_gear_param.kCompress = 5400.0 * 4.4482 / 0.3048;
	right_gear_param.dRebound = 320.0 * 4.4482 / 0.3048;
	right_gear_param.dCompress = 160 * 4.4482 / 0.3048;
	right_gear_param.GearDirection(mathauxiliary::VECTOR_X) = 0.0;
	right_gear_param.GearDirection(mathauxiliary::VECTOR_Y) = 0.0;
	right_gear_param.GearDirection(mathauxiliary::VECTOR_Z) = 1.0;
	right_gear_param.GearPosition(mathauxiliary::VECTOR_X) = -0.4369;
	right_gear_param.GearPosition(mathauxiliary::VECTOR_Y) = 1.2763;
	right_gear_param.GearPosition(mathauxiliary::VECTOR_Z) = 1.3960;
	right_gear_param.isSteering = false;
	unsigned int right_gear = SimInstance1.AddSubSystem(right_gear_param);


	mathblocks::SumParameter total_gear_force_param;
	total_gear_force_param.input_dimensions = 3;
	total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
	total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
	total_gear_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);

	unsigned int total_gear_force = SimInstance1.AddSubSystem(total_gear_force_param);
	// same parameter
	unsigned int total_gear_moment = SimInstance1.AddSubSystem(total_gear_force_param);

	mathblocks::SumParameter total_force_param;
	total_force_param.input_dimensions = 3;
	total_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
	total_force_param.SignList.push_back(mathblocks::SUM_POSITIVE);
	unsigned int total_force = SimInstance1.AddSubSystem(total_force_param);

	unsigned int total_moment = SimInstance1.AddSubSystem(total_force_param);

	// to body-fixed frame

	mathblocks::MultiplicationParam gear_force_to_inertial_param;
	gear_force_to_inertial_param.input1_dimension(mathblocks::MATRIX_ROW) = 3;
	gear_force_to_inertial_param.input1_dimension(mathblocks::MATRIX_COL) = 3;
	gear_force_to_inertial_param.input2_dimension(mathblocks::MATRIX_ROW) = 3;
	gear_force_to_inertial_param.input2_dimension(mathblocks::MATRIX_COL) = 1;
	gear_force_to_inertial_param.Mode = mathblocks::MULTI_MATRIX;

	unsigned int gear_force_to_inertial = SimInstance1.AddSubSystem(gear_force_to_inertial_param);

	unsigned int gear_moment_to_body = SimInstance1.AddSubSystem(gear_force_to_inertial_param);
	// unsigned int 

	mathblocks::ConstantParameter H_param;

	H_param.value.resize(1, 1);
	H_param.value(0) = -1.0;

	unsigned int H_ground = SimInstance1.AddSubSystem(H_param);

	mathblocks::ConstantParameter Ng_param;
	Ng_param.value.resize(3, 1);
	Ng_param.value(mathauxiliary::VECTOR_X) = 0.0;
	Ng_param.value(mathauxiliary::VECTOR_Y) = 0.0;
	Ng_param.value(mathauxiliary::VECTOR_Z) = - 1.0;

	unsigned int N_ground = SimInstance1.AddSubSystem(Ng_param);

	mathblocks::ConstantParameter gear_swithc_param;
	gear_swithc_param.value.resize(1, 1);
	gear_swithc_param.value(0) = 1.0;
	unsigned int gear_switch = SimInstance1.AddSubSystem(gear_swithc_param);


	double vrelaxation_roll = 0.3;
	double vrelaxation_side = 0.3;
	double vlimit = 5.0;

	double sigma_dynamic = 0.4;
	double sigma_static = 0.5;
	double sigma_roll = 0.15;

	groundcontact::GearLuGreFrictionParameter nose_gear_friction_param;
	nose_gear_friction_param.StiffnessSigma0 = 3.0;
	nose_gear_friction_param.DampingSigma1 = sqrt(nose_gear_friction_param.StiffnessSigma0);

	nose_gear_friction_param.SwitchSigmaD = 1.0 / vrelaxation_roll;
	nose_gear_friction_param.SwitchSigmaS = 1.0 / vrelaxation_roll;

	nose_gear_friction_param.DynamicFrictionCoefficient = sigma_dynamic;
	nose_gear_friction_param.RollingFrictionCoefficient = sigma_roll;
	nose_gear_friction_param.StaticFrictionCoefficient = sigma_static;


	nose_gear_friction_param.vlimit = vlimit;


	unsigned int nose_gear_friction = SimInstance1.AddSubSystem(nose_gear_friction_param);
	unsigned int left_gear_friction = SimInstance1.AddSubSystem(nose_gear_friction_param);
	unsigned int right_gear_friction = SimInstance1.AddSubSystem(nose_gear_friction_param);




	// connections

	// signal output to dynamics:

	/*

	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIx, signal_generator1, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIy, signal_generator2, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIz, signal_generator3, 0);

	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBx, signal_generator1, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBy, signal_generator2, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBz, signal_generator3, 0);
	*/

	SimInstance1.BatchEditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIx, dynamics::DYNAMICS_INPUT_FIz, total_force, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

	SimInstance1.BatchEditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBx, dynamics::DYNAMICS_INPUT_TBz, gear_moment_to_body, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);




	SimInstance1.BatchEditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_OmegaBIx, dynamics::DYNAMICS_INPUT_OmegaBIz, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);

	// dynamics to kinematics
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTx, planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTx);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTy, planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTy);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTz, planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTz);
	SimInstance1.BatchEditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIx, dynamics::KINEMATICS_INPUT_AIz, planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, dynamics::DYNAMICS_OUTPUT_AIz);
	// atmoshpere block
	SimInstance1.EditConnectionMatrix(atmoshpere, 0, height, 0);
	// gravity in body-fixed frame
	SimInstance1.BatchEditConnectionMatrix(product_1, 0, 8, planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
	SimInstance1.BatchEditConnectionMatrix(product_1, 9, 11, gravity, 0, 2);
	// calculate Vb_dot
	SimInstance1.BatchEditConnectionMatrix(crossproduct1, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, dynamics::KINEMATICS_OUTPUT_VBz);
	SimInstance1.BatchEditConnectionMatrix(crossproduct1, 3+mathauxiliary::VECTOR_X, 3+mathauxiliary::VECTOR_Z, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
	SimInstance1.BatchEditConnectionMatrix(product_2, 0, 8, planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
	SimInstance1.BatchEditConnectionMatrix(product_2, 9, 11, planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, dynamics::DYNAMICS_OUTPUT_AIz);
	// vb_dot
	SimInstance1.BatchEditConnectionMatrix(sum_Vb, 0, 2, product_2, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(sum_Vb, 3, 5, crossproduct1, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	// connect the aero angle block
	SimInstance1.BatchEditConnectionMatrix(aeroanlge, aero::AERO_INPUT_R_BI00, aero::AERO_INPUT_R_BI22, planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_HORIZONTALWINDSPEED, horizontal_wind, 0);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_WINDDIRECTION, horizontal_wind, 1);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_VERTICALWINDSPEED, horizontal_wind, 2);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_RHO, fixeddensity, 0);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_SOUNDSPEED, atmoshpere, geographic::AtmSoundSpeed);
	SimInstance1.BatchEditConnectionMatrix(aeroanlge, aero::AERO_INPUT_P, aero::AERO_INPUT_R, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
	SimInstance1.BatchEditConnectionMatrix(aeroanlge, aero::AERO_INPUT_Vbdotx, aero::AERO_INPUT_Vbdotz, sum_Vb, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(aeroanlge, aero::AERO_INPUT_Vbx, aero::AERO_INPUT_Vbz, planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, dynamics::KINEMATICS_OUTPUT_VBz);
	SimInstance1.BatchEditConnectionMatrix(aeroanlge, aero::AERO_INPUT_VIx, aero::AERO_INPUT_VIz, planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);
	// connect the aero force block
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_AOA, aeroanlge, aero::AERO_OUTPUT_AOA);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_SIDESLIP, aeroanlge, aero::AERO_OUTPUT_SIDESLIP);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_Pbar, aeroanlge, aero::AERO_OUTPUT_Pbar);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_Qbar, aeroanlge, aero::AERO_OUTPUT_Qbar);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_Rbar, aeroanlge, aero::AERO_OUTPUT_Rbar);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_MACHNUMBER, aeroanlge, aero::AERO_OUTPUT_MACHNUMBER);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_AOARATE_FILTERED, aeroanlge, aero::AERO_OUTPUT_AOARATE);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_SIDESLIPRATE_FILTERED, aeroanlge, aero::AERO_OUTPUT_SIDESLIPRATE);
	SimInstance1.EditConnectionMatrix(aeroforce, aero::AEROFORCE_INPUT_DYNAMICPRESSURE, aeroanlge, aero::AERO_OUTPUT_DYNAMICPRESSURE);


	// connect the propeller to the shaft dynamics
	SimInstance1.EditConnectionMatrix(propeller_1, propulsionsystem::PROPELLER_INPUT_N, omega2rps, 0);
	SimInstance1.EditConnectionMatrix(propeller_1, propulsionsystem::PROPELLER_INPUT_V, aeroanlge, aero::AERO_OUTPUT_TAS);
	SimInstance1.EditConnectionMatrix(propeller_1, propulsionsystem::PROPELLER_INPUT_RHO, fixeddensity, 0);
	// connect the shaft input to the total torque
	SimInstance1.EditConnectionMatrix(omega2rps, 0, shaftdynamics, 0); // the output of the inertial from rad/s to rps
	SimInstance1.EditConnectionMatrix(shaftdynamics, 0, shaft_inertia, 0);
	SimInstance1.EditConnectionMatrix(shaft_inertia, 0, total_torque_to_shaft, 0);
	SimInstance1.EditConnectionMatrix(total_torque_to_shaft, 0, piston_engine_1, propulsionsystem::PISTONENGINE_OUTPUT_Q);
	SimInstance1.EditConnectionMatrix(total_torque_to_shaft, 1, propeller_1, propulsionsystem::PROPELLER_OUTPUT_Q);
	// connect the piston engine to the shaft

	SimInstance1.EditConnectionMatrix(piston_engine_1, propulsionsystem::PISTONENGINE_INPUT_SHAFTRPS, omega2rps, 0);
	SimInstance1.EditConnectionMatrix(piston_engine_1, propulsionsystem::PISTONENGINE_INPUT_MANIFOLD, fixeddensity, 0);
	SimInstance1.EditConnectionMatrix(piston_engine_1, propulsionsystem::PISTONENGINE_INPUT_THROTTLE, fixedthrottle, 0);
	SimInstance1.EditConnectionMatrix(piston_engine_1, propulsionsystem::PISTONENGINE_INPUT_MIXTURE,  fixedmxiture, 0);
	SimInstance1.EditConnectionMatrix(piston_engine_1, propulsionsystem::PISTONENGINE_INPUT_FUELSTATE,fixefuelstate, 0);

	// connect the cos
	SimInstance1.EditConnectionMatrix(trig_cos1, 0, planekinematics, dynamics::KINEMATICS_OUTPUT_THETADOT);
	SimInstance1.EditConnectionMatrix(trig_cos2, 0, planekinematics, dynamics::KINEMATICS_OUTPUT_EulerRoll);
	SimInstance1.EditConnectionMatrix(saturation, 0, trig_cos2, 0);
	SimInstance1.EditConnectionMatrix(division, 0, trig_cos1, 0);
	SimInstance1.EditConnectionMatrix(division, 1, trig_cos2, 0);

	// connect landing gear

	SimInstance1.EditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_SWITCH, gear_switch, 0);
	SimInstance1.EditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_H, H_ground, 0);
	SimInstance1.EditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
	SimInstance1.BatchEditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, N_ground, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
	SimInstance1.BatchEditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
	SimInstance1.BatchEditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
	SimInstance1.BatchEditConnectionMatrix(nose_gear, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);

	SimInstance1.EditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_SWITCH, gear_switch, 0);
	SimInstance1.EditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_H, H_ground, 0);
	SimInstance1.EditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
	SimInstance1.BatchEditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, N_ground, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
	SimInstance1.BatchEditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
	SimInstance1.BatchEditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
	SimInstance1.BatchEditConnectionMatrix(left_gear, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);

	SimInstance1.EditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_SWITCH, gear_switch, 0);
	SimInstance1.EditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_H, H_ground, 0);
	SimInstance1.EditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_Hdot, simulationcontrol::external, 0);
	SimInstance1.BatchEditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_NGx, groundcontact::GEARNORMAL_INPUT_NGz, N_ground, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_OMEGAbx, groundcontact::GEARNORMAL_INPUT_OMEGAbz, planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, dynamics::KINEMATICS_OUTPUT_OmegaBIz);
	SimInstance1.BatchEditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_PIx, groundcontact::GEARNORMAL_INPUT_PIz, planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, dynamics::KINEMATICS_OUTPUT_XIz);
	SimInstance1.BatchEditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_R_IB00, groundcontact::GEARNORMAL_INPUT_R_IB22, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
	SimInstance1.BatchEditConnectionMatrix(right_gear, groundcontact::GEARNORMAL_INPUT_VIx, groundcontact::GEARNORMAL_INPUT_VIz, planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, dynamics::KINEMATICS_OUTPUT_VIz);
	
	// connect the nose gear friction
	SimInstance1.BatchEditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, nose_gear, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
	SimInstance1.BatchEditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, nose_gear, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
	SimInstance1.BatchEditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, nose_gear, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
	SimInstance1.BatchEditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, nose_gear, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
	SimInstance1.EditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_VpI, nose_gear, groundcontact::GEARNORMAL_OUTPUT_VGIp);
	SimInstance1.EditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_VwI, nose_gear, groundcontact::GEARNORMAL_OUTPUT_VGIw);
	SimInstance1.EditConnectionMatrix(nose_gear_friction, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);

	// connect the left gear friction

	SimInstance1.BatchEditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, left_gear, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
	SimInstance1.BatchEditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, left_gear, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
	SimInstance1.BatchEditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, left_gear, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
	SimInstance1.BatchEditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, left_gear, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
	SimInstance1.EditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_VpI, left_gear, groundcontact::GEARNORMAL_OUTPUT_VGIp);
	SimInstance1.EditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_VwI, left_gear, groundcontact::GEARNORMAL_OUTPUT_VGIw);
	SimInstance1.EditConnectionMatrix(left_gear_friction, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);
	// connect the right gear friction
	SimInstance1.BatchEditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_CPx, groundcontact::LUGREFRICTION_INPUT_CPz, right_gear, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTx, groundcontact::GEARNORMAL_OUTPUT_CONTACTPOINTz);
	SimInstance1.BatchEditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_NIx, groundcontact::LUGREFRICTION_INPUT_NIz, right_gear, groundcontact::GEARNORMAL_OUTPUT_NIx, groundcontact::GEARNORMAL_OUTPUT_NIz);
	SimInstance1.BatchEditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_Npx, groundcontact::LUGREFRICTION_INPUT_Npz, right_gear, groundcontact::GEARNORMAL_OUTPUT_npIx, groundcontact::GEARNORMAL_OUTPUT_npIz);
	SimInstance1.BatchEditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_Nwx, groundcontact::LUGREFRICTION_INPUT_Nwz, right_gear, groundcontact::GEARNORMAL_OUTPUT_nwIx, groundcontact::GEARNORMAL_OUTPUT_nwIz);
	SimInstance1.EditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_VpI, right_gear, groundcontact::GEARNORMAL_OUTPUT_VGIp);
	SimInstance1.EditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_VwI, right_gear, groundcontact::GEARNORMAL_OUTPUT_VGIw);
	SimInstance1.EditConnectionMatrix(right_gear_friction, groundcontact::LUGREFRICTION_INPUT_BREAKING, simulationcontrol::external, 0);



	SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X,       mathauxiliary::VECTOR_Z, nose_gear_friction,  groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);
	SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X + 3 ,  mathauxiliary::VECTOR_Z + 3, left_gear_friction, groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);
    SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X + 6,   mathauxiliary::VECTOR_Z + 6, right_gear_friction, groundcontact::LUGREFRICTION_OUTPUT_FIx, groundcontact::LUGREFRICTION_OUTPUT_FIz);
	//SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, nose_gear, groundcontact::GEARNORMAL_OUTPUT_Nbx, groundcontact::GEARNORMAL_OUTPUT_Nbz);
	//SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X + 3 ,  mathauxiliary::VECTOR_Z + 3, left_gear,  groundcontact::GEARNORMAL_OUTPUT_Nbx, groundcontact::GEARNORMAL_OUTPUT_Nbz);
	//SimInstance1.BatchEditConnectionMatrix(total_gear_force, mathauxiliary::VECTOR_X + 6,   mathauxiliary::VECTOR_Z + 6, right_gear, groundcontact::GEARNORMAL_OUTPUT_Nbx, groundcontact::GEARNORMAL_OUTPUT_Nbz);

	//SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, nose_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, groundcontact::GEARNORMAL_OUTPUT_Mbz);
	//SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X + 3, mathauxiliary::VECTOR_Z +3 , left_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, groundcontact::GEARNORMAL_OUTPUT_Mbz);
	//SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X + 6, mathauxiliary::VECTOR_Z + 6, right_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, groundcontact::GEARNORMAL_OUTPUT_Mbz);

	SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, nose_gear_friction, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);
	SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X + 3, mathauxiliary::VECTOR_Z + 3, left_gear_friction, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);
	SimInstance1.BatchEditConnectionMatrix(total_gear_moment, mathauxiliary::VECTOR_X + 6, mathauxiliary::VECTOR_Z + 6, right_gear_friction, groundcontact::LUGREFRICTION_OUTPUT_MIx, groundcontact::LUGREFRICTION_OUTPUT_MIz);

	//SimInstance1.BatchEditConnectionMatrix(gear_force_to_inertial, 0, 8, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
	//SimInstance1.BatchEditConnectionMatrix(gear_force_to_inertial, 9, 11, total_gear_force, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(gear_moment_to_body, 0, 8, planekinematics, dynamics::KINEMATICS_OUTPUT_R_BI00, dynamics::KINEMATICS_OUTPUT_R_BI22);
	SimInstance1.BatchEditConnectionMatrix(gear_moment_to_body, 9, 11, total_gear_moment, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);

	SimInstance1.BatchEditConnectionMatrix(total_force, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z, total_gear_force, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.BatchEditConnectionMatrix(total_force, mathauxiliary::VECTOR_X+3, mathauxiliary::VECTOR_Z+3, gravity, mathauxiliary::VECTOR_X, mathauxiliary::VECTOR_Z);
	SimInstance1.FlushMakeConnection();

	// define the data log tag

	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_AIx, "AIx");
	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_AIy, "AIy");
	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_AIz, "AIz");
	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTx, "omega_dotx");
	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTy, "omega_doty");
	SimInstance1.DefineDataLogging(planedynamics, dynamics::DYNAMICS_OUTPUT_OMEGA_DOTz, "omega_dotz");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerRoll, "roll");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerPitch, "pitch");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerYaw, "yaw");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, "omegax");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIy, "omegay");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIz, "omegaz");

	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB00, "RWB00");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB01, "RWB01");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB02, "RWB02");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB10, "RWB10");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB11, "RWB11");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB12, "RWB12");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB20, "RWB20");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB21, "RWB21");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_R_WB22, "RWB22");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, "RIB00");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB01, "RIB01");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB02, "RIB02");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB10, "RIB10");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB11, "RIB11");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB12, "RIB12");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB20, "RIB20");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB21, "RIB21");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB22, "RIB22");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VBx, "Vbx");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VBy, "Vby");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VBz, "Vbz");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VIx, "VIx");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VIy, "VIy");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_VIy, "VIz");
	
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_XIx, "XIx");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_XIy, "XIy");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_XIz, "XIz");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_PHIDOT, "phidot");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_THETADOT, "thetadot");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_PSIDOT, "psidot");

	SimInstance1.DefineDataLogging(sum_Vb, mathauxiliary::VECTOR_X, "Vb_dotx");
	SimInstance1.DefineDataLogging(sum_Vb, mathauxiliary::VECTOR_Y, "Vb_doty");
	SimInstance1.DefineDataLogging(sum_Vb, mathauxiliary::VECTOR_Z, "Vb_dotz");

	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmPressure, "pressure");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmSoundSpeed, "soundspeed");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmTemperature, "temperature");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmDensity, "density");

	SimInstance1.DefineDataLogging(product_1, 0, "g0");
	SimInstance1.DefineDataLogging(product_1, 1, "g1");
	SimInstance1.DefineDataLogging(product_1, 2, "g2");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_TAS, "TAS");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_AOA, "AOA");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_AOARATE, "AOArate");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_SIDESLIP, "Sideslip");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_SIDESLIPRATE, "Sidesliprate");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_DYNAMICPRESSURE, "dynamicpressure");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_Pbar, "Pbar");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_Qbar, "Qbar");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_Rbar, "Rbar");
	SimInstance1.DefineDataLogging(aeroanlge, aero::AERO_OUTPUT_GAMMA, "gamma");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_FBx, "FBx");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_FBy, "FBy");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_FBz, "FBz");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_MBx, "MBx");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_MBy, "MBy");
	SimInstance1.DefineDataLogging(aeroforce, aero::AEROFORCE_OUTPUT_MBz, "MBz");

	SimInstance1.DefineDataLogging(propeller_1, propulsionsystem::PROPELLER_OUTPUT_T, "Thrust");
	SimInstance1.DefineDataLogging(propeller_1, propulsionsystem::PROPELLER_OUTPUT_Q, "TorqueRequired");
	SimInstance1.DefineDataLogging(propeller_1, propulsionsystem::PROPELLER_OUTPUT_CT, "CT");
	SimInstance1.DefineDataLogging(propeller_1, propulsionsystem::PROPELLER_OUTPUT_CP, "CP");
	SimInstance1.DefineDataLogging(omega2rps, 0, "Shaftrps");
	SimInstance1.DefineDataLogging(piston_engine_1, propulsionsystem::PISTONENGINE_OUTPUT_Q, "TorqueAvaliable");
	SimInstance1.DefineDataLogging(piston_engine_1, propulsionsystem::PISTONENGINE_OUTPUT_FUELRATE, "FuelRate");

	SimInstance1.DefineDataLogging(division, 0, "ans_1");

	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSION, "Snose");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSION, "Sleft");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSION, "Sright");

	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSIONRATE, "Sdotnose");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSIONRATE, "Sdotleft");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_COMPRESSIONRATE, "Sdotright");


	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Nbx, "NoseGearForceX");
	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Nby, "NoseGearForceY");
	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Nbz, "NoseGearForceZ");

	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Nbx, "LeftGearForceX");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Nby, "LeftGearForceY");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Nbz, "LeftGearForceZ");

	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Nbx, "RightGearForceX");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Nby, "RightGearForceY");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Nbz, "RightGearForceZ");

	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, "NoseGearMomentX");
	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Mby, "NoseGearMomentY");
	SimInstance1.DefineDataLogging(nose_gear, groundcontact::GEARNORMAL_OUTPUT_Mbz, "NoseGearMomentZ");

	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, "LeftGearMomentX");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Mby, "LeftGearMomentY");
	SimInstance1.DefineDataLogging(left_gear, groundcontact::GEARNORMAL_OUTPUT_Mbz, "LeftGearMomentZ");

	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Mbx, "RightGearMomentX");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Mby, "RightGearMomentY");
	SimInstance1.DefineDataLogging(right_gear, groundcontact::GEARNORMAL_OUTPUT_Mbz, "RightGearMomentZ");


	// SimInstance1.DisplayLoggerTagList();// show the logged tags

	SimInstance1.DisplaySystemParameter(aeroforce);

	bool flag = SimInstance1.PreRunProcess();

	SimInstance1.DisplayExternalInputMapping(aeroanlge);

	int N_steps = 2000;

	if (flag) { // if successful, run updates
		std::cout << " All good, ready to run. Enter 1 to start.../ Enter other number to stop ..." << std::endl;
		int start = 0;
		std::cin >> start;
		if (start != 1) {
			std::cout << " Abort... " << std::endl;
			return 0;
		}
		std::cout << "Running ... " << std::endl;
		VectorXd extern_input;
		SimInstance1.ReshapeExternalInputVector(extern_input);
		t = clock();
		for (int i = 0; i < N_steps; i++)
		{
			SimInstance1.Run_Update(extern_input);
		}
		t = clock() - t;
		std::cout << "Calculation Takes " << t << " clicks " << ((float)t) / CLOCKS_PER_SEC << " seconds \n";
	}
	else {
		std::cout << "Initialization failed, check subsystem connections" << std::endl;
	}

	SimInstance1.PostRunProcess();
#endif // DEBUG
	getchar();
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
