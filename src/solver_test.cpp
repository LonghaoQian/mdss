// solver_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
#include "util_Recorder.h"
#include "MatlabIO.h"
using std::iostream;

#define DEBUG_RIGID_BODY
//#define DEBUG_LINEAR
//#define DEBUG_STIFF_MODEL
//#define DEBUG_DISP
int main()
{
	SolverConfig config1;
	config1.eposilon = 0.00001;
	config1.adaptive_step = false;
	config1.frame_step = 0.02;
	config1.mim_step = 0.005;
	config1.start_time = 0.0;
	config1.solver_type = RungeKuttaFamily::RUNGKUTTA45;
	SimController SimInstance1(config1);

#ifdef DEBUG_RIGID_BODY
	// debug using rigid body
	RigidBodyParameter rigid_1;
	RigidBodyCondition rigid_1_IC;
	rigid_1.J << 1, 0, 0,
		         0, 1, 0,
		         0, 0, 1;

	rigid_1.m = 1;

	rigid_1_IC.Euler << 0, 0, 0;
	rigid_1_IC.Omega_BI << 0, 0, 0;
	rigid_1_IC.V_I << 0, 0, 0;
	rigid_1_IC.X_I << 0, 0, 0;

	source_sink::SignalGeneratorparameter para_sinewave_1;
	para_sinewave_1.amplitude = 1.0;
	para_sinewave_1.frequency = 0.5;
	para_sinewave_1.num_of_channels = 3;
	para_sinewave_1.phase_shift = 0.0;
	para_sinewave_1.waveshape = source_sink::SINE;

	subsystem_handle rigid_body_1 = SimInstance1.AddSubSystem(rigid_1, rigid_1_IC); //0
	subsystem_handle sinewave_1 = SimInstance1.AddSubSystem(para_sinewave_1);
	std::cout << rigid_body_1.ID << std::endl;
	std::cout << sinewave_1.ID << std::endl;
	//////////////////////////MAKE CONNECTIONs///////////////////////////////////////
	SIMCONNECTION Connection_Rigidbody;

	Connection_Rigidbody.resize(6, 2);
	Connection_Rigidbody(0, 0) = EXTERNAL_INPUT;
	Connection_Rigidbody(0, 1) = 0;

	Connection_Rigidbody(1, 0) = EXTERNAL_INPUT;
	Connection_Rigidbody(1, 1) = 0;

	Connection_Rigidbody(2, 0) = EXTERNAL_INPUT;
	Connection_Rigidbody(2, 1) = 0;

	Connection_Rigidbody(3, 0) = sinewave_1.ID;
	Connection_Rigidbody(3, 1) = 0;

	Connection_Rigidbody(4, 0) = sinewave_1.ID;
	Connection_Rigidbody(4, 1) = 1;

	Connection_Rigidbody(5, 0) = sinewave_1.ID;
	Connection_Rigidbody(5, 1) = 2;


	SimInstance1.MakeConnection(rigid_body_1.ID, Connection_Rigidbody);

	bool flag = SimInstance1.PreRunProcess(); 
	MatlabIO Recorder;
	int N_steps = 500;

	MatrixXd matdata;
	matdata.resize(N_steps+1, 28); // TIME 0  OUTPUT 1-21 F_B 22-24 M_B 25-27
	matdata.setZero();// reset the buffer to zero

	if (flag) { // if successful, run updates
	
		VectorXd extern_input;
		SimInstance1.ReshapeExternalInputVector(extern_input);
		for (int i = 0; i < N_steps; i++)
		{
			matdata(i, 0) = SimInstance1.Run_GetSystemTime();
			// save F_B
			matdata(i, 22) = extern_input(0);
			matdata(i, 23) = extern_input(1);
			matdata(i, 24) = extern_input(2);
			// save M_B
			matdata(i, 25) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(0);
			matdata(i, 26) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(1);
			matdata(i, 27) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(2);
			for (int j = 0; j < 21; j++) {
				matdata(i, j + 1) = SimInstance1.Run_GetSubsystemOuput(0)(j);
			}

			SimInstance1.Run_Update(extern_input);
		}
		matdata(N_steps, 0) = SimInstance1.Run_GetSystemTime();
		for (int j = 0; j < 21; j++) {
			matdata(N_steps, j + 1) = SimInstance1.Run_GetSubsystemOuput(0)(j);
		}

		// save F_B
		matdata(N_steps, 22) = extern_input(0);
		matdata(N_steps, 23) = extern_input(1);
		matdata(N_steps, 24) = extern_input(2);
		// save M_B
		matdata(N_steps, 25) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(0);
		matdata(N_steps, 26) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(1);
		matdata(N_steps, 27) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(2);

		if (Recorder.SaveToMatFile(matdata, "rigid_body_state.mat", "A")) {
			std::cout << "successfuly save the data" << std::endl;
		}
		else {
			std::cout << " failed save the data" << std::endl;
		}
	}
	else {
		std::cout << "Initialization failed, check subsystem connections" << std::endl;
	}


#endif // DEBUG_RIGID_BODY

#ifdef DEBUG_ALL
	geographic::StandardAtmosphereParameter atm_block_para;
	atm_block_para.atmoshpere_name_ = "atmosphere_data.mat";

	aero::AerosForceParameter AERO1;
	AERO1.S = 16.16; //m^2
	AERO1.b_ = 10.9728;//m
	AERO1.c_bar_ = 1.4935;//m
	AERO1.aero_reference_point_ << 0,0,-0.1829;
	AERO1.aeroparam_.CL0_ = 0.27;
	AERO1.aeroparam_.CLadot_ = 1.7;
	AERO1.aeroparam_.CLq_ = 3.9;
	AERO1.aeroparam_.CLde_ = -0.24;
	AERO1.aeroparam_.CL_alpha_ = 4.66;

	AERO1.aeroparam_.CD0_ = 0.032;
	AERO1.aeroparam_.CDbeta_ = 0.17;
	AERO1.aeroparam_.CDde_ = 0.06;
	AERO1.aeroparam_.CDDf_ = 0.0007;
	AERO1.aeroparam_.CD_alpha_ = 0.0051894;

	AERO1.aeroparam_.CYb_ = -0.3095;
	AERO1.aeroparam_.CYda_ = -0.05;
	AERO1.aeroparam_.CYdr_ = 0.098;
	AERO1.aeroparam_.CYp_ = -0.037;
	AERO1.aeroparam_.CYr_ = 0.21;

	AERO1.aeroparam_.Clb_ = -0.0891;
	AERO1.aeroparam_.Clda_ = 0.23;
	AERO1.aeroparam_.Clp_ = -0.47;
	AERO1.aeroparam_.Clr_ = 0.12;
	AERO1.aeroparam_.Cldr_ = 0.0147;

    AERO1.aeroparam_.Cm0_ = 0.12;
	AERO1.aeroparam_.Cmadot_ = -5.2;
	AERO1.aeroparam_.Cmalpha_ = -1.8;
	AERO1.aeroparam_.Cmde_ = -1.18;
	AERO1.aeroparam_.CmDf_ = -0.03;
	AERO1.aeroparam_.Cmq_ = -12.4;
	
	AERO1.aeroparam_.Cnb_ = 0.0650;
	AERO1.aeroparam_.Cnda_ = 0.0053;
	AERO1.aeroparam_.Cndr_ = -0.043;
	AERO1.aeroparam_.Cnp_ = -0.03;
	AERO1.aeroparam_.Cnr_ = -0.099;

	aero::AeroAngleParameter aero_angle;

	aero_angle.b_ = AERO1.b_;
	aero_angle.c_bar_ = AERO1.c_bar_;
	aero_angle.min_airspeed_ = 1;



	geographic::GravityModelParameter gravityparam;
	gravityparam.Mode = geographic::FlatGround;

	RigidBodyParameter rigid_1;
	RigidBodyCondition rigid_1_IC;
	rigid_1.J <<1285.3, 0, 0,
		        0,1824.9,0,
				0,0,2666.9;

	rigid_1.m = 962.7037;

	rigid_1_IC.Euler << 0, 2/57.3, 0;
	rigid_1_IC.Omega_BI << 0, 0, 0;
	rigid_1_IC.V_I << 52.62, 0, 0;
	rigid_1_IC.X_I << 10, 0, -1524;

	mathblocks::ConstantParameter thrust;
	thrust.value.resize(3);
	thrust.value << 200, 0, 0;

	mathblocks::SumParameter sum_1;

	sum_1.input_dimensions = 3;
	sum_1.num_of_inputs = 2;

	mathblocks::MultiplicationParam multiple1;

	multiple1.input1_dimension << 3, 3;
	multiple1.input2_dimension << 3, 1;
	multiple1.Mode = mathblocks::Matrix;

	mathblocks::GainParameter gain1;
	gain1.K.resize(1, 1);
	gain1.K(0, 0) = -1;
	gain1.Mode = mathblocks::Scalar;
	gain1.num_of_inputs = 1;

	SimInstance1.AddSubSystem(atm_block_para);// 0 subsystem 
	SimInstance1.AddSubSystem(AERO1);	      // 1 subsystem 
	SimInstance1.AddSubSystem(aero_angle);    // 2 
	SimInstance1.AddSubSystem(gravityparam);  // 3
	SimInstance1.AddSubSystem(rigid_1, rigid_1_IC); // 4
	SimInstance1.AddSubSystem(thrust); // 5
	SimInstance1.AddSubSystem(sum_1);  // 6
	SimInstance1.AddSubSystem(sum_1);   // 7
	SimInstance1.AddSubSystem(multiple1); // 8
	SimInstance1.AddSubSystem(gain1);     // 9

	// define subsystem connections
	SIMCONNECTION Connection_Atmosphere,
		Connection_AeroForce,
		Connection_AeroAngle,
		Connection_Gravity, // No coonection 
		Connection_Rigidbody,
		Connection_Thrust,// No coonection
		Connection_Sum_1,
		Connection_Sum_2,
		Connection_Multi_1,
		Connection_Gain_1;
	// index of the input port, system ID of the output, index of the output 
	Connection_Atmosphere.resize(1, 2);
	Connection_Atmosphere(0, 0) = 9;
	Connection_Atmosphere(0, 1) = 0;
	SimInstance1.MakeConnection(0, Connection_Atmosphere);

	Connection_AeroForce.resize(16, 2);
	Connection_AeroForce(0, 0) = 2;
	Connection_AeroForce(0, 1) = 4;

	Connection_AeroForce(1, 0) = 2;
	Connection_AeroForce(1, 1) = 2;

	Connection_AeroForce(2, 0) = 2;
	Connection_AeroForce(2, 1) = 3;

	Connection_AeroForce(3, 0) = 2;
	Connection_AeroForce(3, 1) = 7;

	Connection_AeroForce(4, 0) = 2;
	Connection_AeroForce(4, 1) = 8;

	Connection_AeroForce(5, 0) = 2;
	Connection_AeroForce(5, 1) = 9;

	Connection_AeroForce(6, 0) = 2;
	Connection_AeroForce(6, 1) = 5;

	Connection_AeroForce(7, 0) = 2;
	Connection_AeroForce(7, 1) = 6;

	Connection_AeroForce(8, 0) = -1;
	Connection_AeroForce(8, 1) = 0;

	Connection_AeroForce(9, 0) = -1;
	Connection_AeroForce(9, 1) = 0;

	Connection_AeroForce(10, 0) = -1;
	Connection_AeroForce(10, 1) = 0;

	Connection_AeroForce(11, 0) = -1;
	Connection_AeroForce(11, 1) = 0;

	Connection_AeroForce(12, 0) = -1;
	Connection_AeroForce(12, 1) = 0;

	Connection_AeroForce(13, 0) = -1;
	Connection_AeroForce(13, 1) = 0;

	Connection_AeroForce(14, 0) = 2;
	Connection_AeroForce(14, 1) = 1;

	Connection_AeroForce(15, 0) = 9;
	Connection_AeroForce(15, 1) = 0;
	SimInstance1.MakeConnection(1, Connection_AeroForce);

	Connection_AeroAngle.resize(11, 2);
	Connection_AeroAngle(0, 0) = 0;
	Connection_AeroAngle(0, 1) = 3;

	Connection_AeroAngle(1, 0) = 0;
	Connection_AeroAngle(1, 1) = 1;

	Connection_AeroAngle(2, 0) = 4;
	Connection_AeroAngle(2, 1) = 18;

	Connection_AeroAngle(3, 0) = 4;
	Connection_AeroAngle(3, 1) = 19;

	Connection_AeroAngle(4, 0) = 4;
	Connection_AeroAngle(4, 1) = 20;

	Connection_AeroAngle(5, 0) = 4;
	Connection_AeroAngle(5, 1) = 3;

	Connection_AeroAngle(6, 0) = 4;
	Connection_AeroAngle(6, 1) = 4;

	Connection_AeroAngle(7, 0) = 4;
	Connection_AeroAngle(7, 1) = 5;

	Connection_AeroAngle(8, 0) = -1;
	Connection_AeroAngle(8, 1) = 0;

	Connection_AeroAngle(9, 0) = -1;
	Connection_AeroAngle(9, 1) = 0;

	Connection_AeroAngle(10, 0) = -1;
	Connection_AeroAngle(10, 1) = 0;
	SimInstance1.MakeConnection(2, Connection_AeroAngle);

	Connection_Rigidbody.resize(6, 2);
	Connection_Rigidbody(0, 0) = 6;
	Connection_Rigidbody(0, 1) = 0;

	Connection_Rigidbody(1, 0) = 6;
	Connection_Rigidbody(1, 1) = 1;

	Connection_Rigidbody(2, 0) = 6;
	Connection_Rigidbody(2, 1) = 2;

	Connection_Rigidbody(3, 0) = 1;
	Connection_Rigidbody(3, 1) = 3;

	Connection_Rigidbody(4, 0) = 1;
	Connection_Rigidbody(4, 1) = 4;

	Connection_Rigidbody(5, 0) = 1;
	Connection_Rigidbody(5, 1) = 5;

	SimInstance1.MakeConnection(4, Connection_Rigidbody);

	Connection_Sum_1.resize(6, 2);
	Connection_Sum_1(0, 0) = 8;
	Connection_Sum_1(0, 1) = 0;

	Connection_Sum_1(1, 0) = 8;
	Connection_Sum_1(1, 1) = 1;

	Connection_Sum_1(2, 0) = 8;
	Connection_Sum_1(2, 1) = 2;

	Connection_Sum_1(3, 0) = 7;
	Connection_Sum_1(3, 1) = 0;

	Connection_Sum_1(4, 0) = 7;
	Connection_Sum_1(4, 1) = 1;

	Connection_Sum_1(5, 0) = 7;
	Connection_Sum_1(5, 1) = 2;

	SimInstance1.MakeConnection(6, Connection_Sum_1);

	Connection_Sum_2.resize(6,2);
	Connection_Sum_2(0, 0) = 1;
	Connection_Sum_2(0, 1) = 0;

	Connection_Sum_2(1, 0) = 1;
	Connection_Sum_2(1, 1) = 1;

	Connection_Sum_2(2, 0) = 1;
	Connection_Sum_2(2, 1) = 2;

	Connection_Sum_2(3, 0) = 5;
	Connection_Sum_2(3, 1) = 0;

	Connection_Sum_2(4, 0) = 5;
	Connection_Sum_2(4, 1) = 1;

	Connection_Sum_2(5, 0) = 5;
	Connection_Sum_2(5, 1) = 2;

	SimInstance1.MakeConnection(7, Connection_Sum_2);

	Connection_Multi_1.resize(12, 2);
	Connection_Multi_1(0, 0) = 4;
	Connection_Multi_1(0, 1) = 9;

	Connection_Multi_1(1, 0) = 4;
	Connection_Multi_1(1, 1) = 10;

	Connection_Multi_1(2, 0) = 4;
	Connection_Multi_1(2, 1) = 11;

	Connection_Multi_1(3, 0) = 4;
	Connection_Multi_1(3, 1) = 12;

	Connection_Multi_1(4, 0) = 4;
	Connection_Multi_1(4, 1) = 13;

	Connection_Multi_1(5, 0) = 4;
	Connection_Multi_1(5, 1) = 14;

	Connection_Multi_1(6, 0) = 4;
	Connection_Multi_1(6, 1) = 15;

	Connection_Multi_1(7, 0) = 4;
	Connection_Multi_1(7, 1) = 16;

	Connection_Multi_1(8, 0) = 4;
	Connection_Multi_1(8, 1) = 17;

	Connection_Multi_1(9, 0) = 3;
	Connection_Multi_1(9, 1) = 0;

	Connection_Multi_1(10, 0) = 3;
	Connection_Multi_1(10, 1) = 1;

	Connection_Multi_1(11, 0) = 3;
	Connection_Multi_1(11, 1) = 2;

	SimInstance1.MakeConnection(8, Connection_Multi_1);

	Connection_Gain_1.resize(1, 2);
	Connection_Gain_1(0, 0) = 4;
	Connection_Gain_1(0, 1) = 8;

	SimInstance1.MakeConnection(9, Connection_Gain_1);

	bool flag = SimInstance1.PreRunProcess();

	int N_steps = 1000;
	MatlabIO Recorder;

	if (flag) { // if successful, run updates
	// print the system info
		VectorXd extern_input;
		extern_input.resize(9);
		extern_input.setZero();
		extern_input(4) = 5.1 / 57.3;
		MatrixXd matdata;
		matdata.resize(N_steps, 4);
		matdata.setZero();// reset the buffer to zero
		for (int i = 0; i < N_steps; i++)
		{
			matdata(i, 0) = SimInstance1.Run_GetSystemTime();// 0 is the system time
			matdata(i, 1) = SimInstance1.Run_GetSubsystemOuput(2)(0);
			matdata(i, 2) = - SimInstance1.Run_GetSubsystemOuput(4)(8);
			matdata(i, 3) = SimInstance1.Run_GetSubsystemOuput(2)(2);
			std::cout << "t= " << matdata(i, 0) << std::endl;
			std::cout << "TAS = " << matdata(i, 1) << " m/s" <<std::endl;
			std::cout << "VI_x = " << SimInstance1.Run_GetSubsystemOuput(4)(0) << " m/s, VI_y = "
									<< SimInstance1.Run_GetSubsystemOuput(4)(1) << " m/s, VI_z = "
								<< SimInstance1.Run_GetSubsystemOuput(4)(2) << std::endl;
			std::cout << " X = " << SimInstance1.Run_GetSubsystemOuput(4)(6) << " m, Y =  " << SimInstance1.Run_GetSubsystemOuput(4)(7) << " m " << endl;
			std::cout << "Altitude  = " << matdata(i, 2) << "m" << std::endl;
			std::cout << "Alpha = " << 57.3*matdata(i, 3) << " DEG " << std::endl;
			std::cout << "Beta = " << 57.3*SimInstance1.Run_GetSubsystemOuput(2)(3) << " DEG " << std::endl;
			std::cout << "Dynamic Pressure = " << 57.3*SimInstance1.Run_GetSubsystemOuput(2)(4) << " Pa " << std::endl;
			std::cout << "rho = " << SimInstance1.Run_GetSubsystemOuput(0)(3) << " kg/m^3, a = " << SimInstance1.Run_GetSubsystemOuput(0)(1) << " m/s " << endl;
			//std::cout << "Dynamic Pressure = " << 57.3*SimInstance1.Run_GetSubsystemOuput(2)(4) << " Pa " << std::endl;
			std::cout << "----------------------------------------------" << std::endl;
			SimInstance1.Run_Update(extern_input);
		}


		if (Recorder.SaveToMatFile(matdata, "recorded_state.mat", "A")) {
			std::cout << "successfuly save the data" << std::endl;
		}
		else {
			std::cout << " failed save the data" << std::endl;
		}
	}
	else {
		std::cout << "Initialization failed, check subsystem connections" << std::endl;
	}
#endif 
	std::cout << "Calculationi Finished " << std::endl;
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
