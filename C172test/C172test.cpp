// C172test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
int main()
{
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
	// 
	double planemass = 700.0;
	// define kinematics
	dynamics::RigidBodyKinematicsInitialCondition initial_condition;
	initial_condition.Euler0(0) = 0.0;
	initial_condition.Euler0(1) = 5.0/57.3;
	initial_condition.Euler0(2) = 30.0/57.3;
	initial_condition.Omega0(0) = 0.0;
	initial_condition.Omega0(1) = 0.0;
	initial_condition.Omega0(2) = 0.0;
	initial_condition.VI0(0) = 40.0;
	initial_condition.VI0(1) = 0.0;
	initial_condition.VI0(2) = 0.0;
	initial_condition.XI0(0) = 0.0;
	initial_condition.XI0(1) = 0.0;
	initial_condition.XI0(2) = -100; // END frame
	unsigned int planekinematics = SimInstance1.AddSubSystem(initial_condition);
	// define dynamics
	dynamics::RigidBodyDynamicsParamter dynamics_parameter;
	dynamics_parameter.J(0, 0) = 1285.31541660000; // kg m^2
	dynamics_parameter.J(1, 1) = 1824.93096070000;
	dynamics_parameter.J(2, 2) = 2666.89390765000;
	dynamics_parameter.m = planemass; //kg
	unsigned int planedynamics = SimInstance1.AddSubSystem(dynamics_parameter);
	// define gravity 
	mathblocks::ConstantParameter gravity;
	gravity.value.resize(3, 1);
	gravity.value(0) = 0.0;
	gravity.value(1) = 0.0;
	gravity.value(2) = 9.81*planemass;
	unsigned int gravity = SimInstance1.AddSubSystem(gravity);
	// define aeroangle
	aero::AeroAngleParameter aeroangleparameter;
	aeroangleparameter.b_ = 10.097;
	aeroangleparameter.c_bar_ = 1.493520000000000;
	aeroangleparameter.min_airspeed_ = 0.2;
	unsigned int aeroanlge = SimInstance1.AddSubSystem(aeroangleparameter);
	// force summation
	mathblocks::SumParameter force_sum_param;
	force_sum_param.input_dimensions = 3;
	force_sum_param.num_of_inputs = 3; // areo gravity and thrust
	unsigned int force_summation = SimInstance1.AddSubSystem(force_sum_param);
	//
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIx, planedynamics, dynamics::DYNAMICS_OUTPUT_AIx);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIy, planedynamics, dynamics::DYNAMICS_OUTPUT_AIy);
	SimInstance1.EditConnectionMatrix(planekinematics, dynamics::KINEMATICS_INPUT_AIz, planedynamics, dynamics::DYNAMICS_OUTPUT_AIz);

	SimInstance1.EditConnectionMatrix(planedynamics, dynamics::DYNAMICS_INPUT_FIx, );

	// force summation : aero, gravity, and thrust 


	// define aeroforces

	// define engine


	// flush connection matrix
	SimInstance1.FlushMakeConnection();

	// define the data log tag

	SimInstance1.DefineDataLogging(speed, 0, "airspeed");
	SimInstance1.DefineDataLogging(position, 0, "AOA");
	SimInstance1.DefineDataLogging(gain_3, 0, "Beta");

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
