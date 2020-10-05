// C172test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
#include <time.h>
int main()
{
	
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
	// plane mass
	double planemass = 700.0;
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
	initial_condition.XI0(mathauxiliary::VECTOR_Z) = -100; // END frame
	unsigned int planekinematics = SimInstance1.AddSubSystem(initial_condition);
	//// define dynamics
	//dynamics::RigidBodyDynamicsParamter dynamics_parameter;
	//dynamics_parameter.J(0, 0) = 1285.31541660000; // kg m^2
	//dynamics_parameter.J(1, 1) = 1824.93096070000;
	//dynamics_parameter.J(2, 2) = 2666.89390765000;
	//dynamics_parameter.m = planemass; //kg
	//unsigned int planedynamics = SimInstance1.AddSubSystem(dynamics_parameter);
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

	//// define aeroangle
	aero::AeroAngleParameter aeroangleparameter;
	aeroangleparameter.b_ = 10.097;
	aeroangleparameter.c_bar_ = 1.493520000000000;
	aeroangleparameter.min_airspeed_ = 0.2;
	unsigned int aeroanlge = SimInstance1.AddSubSystem(aeroangleparameter);

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
	//// 
	//SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIx, planedynamics, dynamics::DYNAMICS_OUTPUT_AIx);
	//SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIy, planedynamics, dynamics::DYNAMICS_OUTPUT_AIy);
	//SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIz, planedynamics, dynamics::DYNAMICS_OUTPUT_AIz);

	//SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIx, force_summation, mathauxiliary::VECTOR_X);
	//SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIy, force_summation, 1);
	//SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIz, force_summation, 2);


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

	// define aeroforces

	// define engine

	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTx, signal_generator1, 0);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTy, signal_generator2, 0);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_OMEGA_DOTz, signal_generator3, 0);
	SimInstance1.EditConnectionMatrix(atmoshpere, 0, height, 0);

	SimInstance1.BatchEditConnectionMatrix(product_1, 0, 8, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, dynamics::KINEMATICS_OUTPUT_R_IB22);
	SimInstance1.BatchEditConnectionMatrix(product_1, 9, 11, gravity, 0, 2);

	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_RHO, atmoshpere, geographic::AtmDensity);
	SimInstance1.EditConnectionMatrix(aeroanlge, aero::AERO_INPUT_SOUNDSPEED, atmoshpere, geographic::AtmSoundSpeed);
	

	//SimInstance1.EditConnectionMatrix(product_1, 0, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00);
	//SimInstance1.EditConnectionMatrix(product_1, 1, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB10);
	//SimInstance1.EditConnectionMatrix(product_1, 2, planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB20);

		// flush connection matrix
	SimInstance1.FlushMakeConnection();

	// define the data log tag

	SimInstance1.DefineDataLogging(signal_generator1, 0, "omega_dotx");
	SimInstance1.DefineDataLogging(signal_generator2, 0, "omega_doty");
	SimInstance1.DefineDataLogging(signal_generator3, 0, "omega_dotz");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerRoll, "roll");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerPitch, "pitch");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_EulerYaw, "yaw");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIx, "omegax");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIy, "omegay");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_OmegaBIz, "omegaz");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB00, "RIB00");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB01, "RIB01");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB02, "RIB02");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB10, "RIB10");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB11, "RIB11");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB12, "RIB12");

	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB20, "RIB20");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB21, "RIB21");
	SimInstance1.DefineDataLogging(planekinematics, dynamics::KINEMATICS_OUTPUT_R_IB22, "RIB22");

	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmPressure, "pressure");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmSoundSpeed, "soundspeed");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmTemperature, "temperature");
	SimInstance1.DefineDataLogging(atmoshpere, geographic::AtmDensity, "density");

	SimInstance1.DefineDataLogging(product_1, 0, "g0");
	SimInstance1.DefineDataLogging(product_1, 1, "g1");
	SimInstance1.DefineDataLogging(product_1, 2, "g2");

	SimInstance1.DisplayLoggerTagList();// show the logged tags
	bool flag = SimInstance1.PreRunProcess();
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
