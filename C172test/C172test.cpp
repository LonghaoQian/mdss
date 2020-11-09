// C172test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
#include "Aircraftmodel.h"
#include <time.h>

// #define DEBUG
int main()
{
	
#ifndef DEBUG

	/*------------- define initial condition --------------------*/
	aircraft::initialcondition C172initialcondition;

	C172initialcondition.engine.propellerRPM = 2200.0; // RPM
	C172initialcondition.plane.omegax = 0.0;
	C172initialcondition.plane.omegay = 0.0;
	C172initialcondition.plane.omegaz = 0.0;
	C172initialcondition.plane.inertialpositionx = 10.0;
	C172initialcondition.plane.inertialpositiony = 10.0;
	C172initialcondition.plane.inertialpositionz = -1000.0;// initial height NED frame
	C172initialcondition.plane.inertialvelocityx = 50.0; // 
	C172initialcondition.plane.inertialvelocityy = 1.0;
	C172initialcondition.plane.inertialvelocityz = 0.5;
	C172initialcondition.plane.roll = 0.0;
	C172initialcondition.plane.pitch = 5.0/57.3;
	C172initialcondition.plane.yaw = 0.0/57.3;

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

	aircraft::C172input controlinput;


	aircraft::AircraftDynamicModel C172aicraftmodel(C172parameter, C172initialcondition);
	int start = 0;
	std::cin >> start;
	if (start != 1) {
		std::cout << " Abort... " << std::endl;
		return 0;
	}
	int N_steps = 1000;
	for (int i = 0; i < N_steps; i++) {
		C172aicraftmodel.UpdateSimulation(controlinput);
	}

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
	double planemass = 1.0;
	// plane CG
	double CG_MAC = 0.25;
	// plane initial condition

	// define kinematics
	dynamics::RigidBodyKinematicsInitialCondition initial_condition;
	initial_condition.Euler0(mathauxiliary::EULER_ROLL) = 0.0;
	initial_condition.Euler0(mathauxiliary::EULER_PITCH) = 5.0/57.3;
	initial_condition.Euler0(mathauxiliary::EULER_YAW) = 30.0/57.3;
	initial_condition.Omega0(mathauxiliary::VECTOR_X) = 0.0;
	initial_condition.Omega0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.Omega0(mathauxiliary::VECTOR_Z) = 0.0;
	initial_condition.VI0(mathauxiliary::VECTOR_X) = 40.0;
	initial_condition.VI0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.VI0(mathauxiliary::VECTOR_Z) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_X) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_Y) = 0.0;
	initial_condition.XI0(mathauxiliary::VECTOR_Z) = 0.0; // END frame
	unsigned int planekinematics = SimInstance1.AddSubSystem(initial_condition);
	// define dynamics
	dynamics::RigidBodyDynamicsParamter dynamics_parameter;
	//dynamics_parameter.J(0, 0) = 1285.31541660000; // kg m^2
	//dynamics_parameter.J(1, 1) = 1824.93096070000;
	//dynamics_parameter.J(2, 2) = 2666.89390765000;
	//dynamics_parameter.m = planemass; //kg
	dynamics_parameter.J(0, 0) = 20.0; // kg m^2
	dynamics_parameter.J(1, 1) = 20.0;
	dynamics_parameter.J(2, 2) = 20.0;
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

	// connections

	// signal output to dynamics:
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIx, signal_generator1, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIy, signal_generator2, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIz, signal_generator3, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBx, signal_generator1, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBy, signal_generator2, 0);
	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_TBz, signal_generator3, 0);
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

	// flush connection matrix
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
