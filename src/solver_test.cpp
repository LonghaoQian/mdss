// solver_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "SimController.h"
#include <time.h> 
using std::iostream;

//#define DEBUG_RIGID_BODY
//#define DEBUG_LINEAR
//#define DEBUG_STIFF_MODEL
//#define DEBUG_DISP
#define DEBUG_ENGINE


#ifdef DEBUG_ENGINE
// FADEC control logic
namespace FADEC{
enum ControlState{
	LOGIC_SHUTDOWN = 0,
	LOGIC_IGNITION,
	LOGIC_STARTING,
	LOGIC_FUELINJECTION,
	LOGIC_NORMAL,
};

struct EngineLogicOutput{
	ControlState state;
	double startertorque;
	double FuelFlow;
};

struct EngineLogicInput {
	bool Ignition;
	bool Starter;
	double N2;
};

class EngineLogic {
public:
	EngineLogic(ControlState initial_state);
	~EngineLogic();
	void ResetState(ControlState reset_state);
	EngineLogicOutput UpdateLogic(const EngineLogicInput& input);
private:
	ControlState state;
	EngineLogicOutput output;
	bool switchflag;
	void OutputFromState(ControlState state_);

	std::map<const ControlState, std::function<ControlState(const EngineLogicInput&)> > SwitchingLogic{
		{LOGIC_SHUTDOWN,[](const EngineLogicInput& input) {
		if (input.Ignition) {
			return LOGIC_IGNITION;
		}
		else {
			return LOGIC_SHUTDOWN;
		}} },
		{LOGIC_IGNITION,[](const EngineLogicInput& input) { 
		if (input.Starter) {
			return LOGIC_STARTING;
		}else {
			return LOGIC_IGNITION;
		}} },
		{LOGIC_STARTING,[](const EngineLogicInput& input) { 
		if (input.N2 > 0.304) {
			return LOGIC_FUELINJECTION;
		}
		else {
			return LOGIC_STARTING;
		}
		} },
		{LOGIC_FUELINJECTION,[](const EngineLogicInput& input) { 
		if (input.N2 > 0.5) {
			return LOGIC_NORMAL;
		}else {
			return LOGIC_FUELINJECTION;
		}
		} },
		{LOGIC_NORMAL,[](const EngineLogicInput& input) {
		if (input.N2 < 0.1) {
			return LOGIC_SHUTDOWN;
		}
		else {
			return LOGIC_NORMAL;
		}} },
	};
};

EngineLogic::EngineLogic(ControlState initial_state) {
	state = initial_state;
	switchflag = false;
}

EngineLogic::~EngineLogic() {

}

void EngineLogic::ResetState(ControlState reset_state) {
	state = reset_state;
}

EngineLogicOutput EngineLogic::UpdateLogic(const EngineLogicInput& input) {
	state = SwitchingLogic[state](input);// update state
	OutputFromState(state);
	return output;
}

void EngineLogic::OutputFromState(ControlState state_) {
	switch (state_) {
		case LOGIC_SHUTDOWN: {
			output.FuelFlow = 0.0;
			output.startertorque = 0.0;
			output.state = LOGIC_SHUTDOWN;
			break;
		}
		case LOGIC_IGNITION: {
			output.FuelFlow = 0.0;
			output.startertorque = 0.0;
			output.state = LOGIC_IGNITION;
			break;
		}
		case LOGIC_STARTING: {
			output.FuelFlow = 0.0;
			output.startertorque = 1.0;
			output.state = LOGIC_STARTING;
			break;
		}
		case LOGIC_FUELINJECTION: {
			output.FuelFlow = 1.0;
			output.startertorque = 1.0;
			output.state = LOGIC_FUELINJECTION;
			break;
		}
		case LOGIC_NORMAL: {
			output.FuelFlow = 1.0;
			output.startertorque = 0.0;
			output.state = LOGIC_NORMAL;
			break;
		}
	}
}

}
#endif
int main()
{
	simulationcontrol::SolverConfig config1;
	config1.eposilon = 0.00001;
	config1.adaptive_step = false;
	config1.frame_step = 0.02;
	config1.mim_step = 0.005;
	config1.start_time = 0.0;
	config1.solver_type = RungeKuttaFamily::DORMANDPRINCE;
	config1.loggingconfig.filename = "datalog.txt";
	config1.loggingconfig.uselogging = false;
	simulationcontrol::SimController SimInstance1(config1);

	clock_t t;



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

	source_sink::PeriodicWaveparameter para_sinewave_1;
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


#ifdef DEBUG_LINEAR

	/*
	A = [-3.8916 * 10 ^ -2 18.9920 - 32.139 0.0000;
	-1.0285 * 10 ^ -3 - 0.64537 5.62290 * 10 ^ -3 1.0000;
	0.0000 0.0000 0.0000 1.0000;
	8.08470 * 10 ^ -5 - 0.77287 - 8.0979 * 10 ^ -4 - 0.52900];
	B = [0; 0; 0; 0.010992*57.3];	
	*/
	linearsystem::TransferFunctionParameter tf_1_para;

	tf_1_para.Denominator.resize(6);
	tf_1_para.Numerator.resize(6);
	tf_1_para.Numerator(0) = 0.23;
	tf_1_para.Numerator(1) = 0.35;
	tf_1_para.Numerator(2) = 0.32;
	tf_1_para.Numerator(3) = -0.45;
	tf_1_para.Numerator(4) = 0.2;
	tf_1_para.Numerator(5) = 1.0;

	tf_1_para.Denominator(0) = 2.0;
	tf_1_para.Denominator(1) = 0.29;
	tf_1_para.Denominator(2) = -0.0082;
	tf_1_para.Denominator(3) = 3.0;
	tf_1_para.Denominator(4) = 26.0;
	tf_1_para.Denominator(5) = 0.1;

	linearsystem::LTIParameter lti_1;
	linearsystem::LTIInitialCondition lti_1_IC;
	lti_1.A.resize(4, 4);
	lti_1.B.resize(4, 1);
	lti_1.C.resize(4, 4);
	lti_1.D.resize(4, 1);
	lti_1.A << -3.8916 * pow(10, -2), 18.9920, -32.139, 0.0000,
		-1.0285 * pow(10, -3), -0.64537, 5.62290 * pow(10, -3), 1.0000,
		0.0000, 0.0000, 0.0000, 1.0000,
		8.08470 * pow(10,-5), - 0.77287, - 8.0979 * pow(10,-4), - 0.52900;
	lti_1.B << 0.0,
		0.0,
		0.0,
		0.010992*57.3;
	lti_1.C.setIdentity();
	lti_1.D.setZero();
	lti_1_IC.X_0.resize(4, 1);
	lti_1_IC.X_0.setZero();
	lti_1_IC.X_0(1) = 1.0;

	source_sink::PeriodicWaveparameter para_sinewave_1;
	para_sinewave_1.amplitude = 1.0;
	para_sinewave_1.frequency = 0.5;
	para_sinewave_1.num_of_channels = 1;
	para_sinewave_1.phase_shift = 0.0;
	para_sinewave_1.waveshape = source_sink::SINE;

	mathblocks::GainParameter gain_1_para;
	gain_1_para.Mode = mathblocks::ElementWise;
	gain_1_para.K.resize(1, 1);
	gain_1_para.K(0, 0) = 2.0;
	gain_1_para.num_of_inputs = 1;

	mathblocks::SumParameter sum_1_para;

	sum_1_para.input_dimensions = 1;
	sum_1_para.num_of_inputs = 2;
	sum_1_para.sign_list.resize(2);
	sum_1_para.sign_list(0) = 1.0;
	sum_1_para.sign_list(1) = -1.0;

	mathblocks::GainParameter gain_2_para;
	gain_2_para.Mode = mathblocks::ElementWise;
	gain_2_para.K.resize(1, 1);
	gain_2_para.K(0, 0) = 1.0;
	gain_2_para.num_of_inputs = 1;
	//subsystem_handle tf_1 = SimInstance1.AddSubSystem(tf_1_para);
	subsystem_handle sinewave_1 = SimInstance1.AddSubSystem(para_sinewave_1);
	subsystem_handle LTI_1 = SimInstance1.AddSubSystem(lti_1, lti_1_IC);
	subsystem_handle Gain_1 = SimInstance1.AddSubSystem(gain_1_para);
	subsystem_handle Gain_2 = SimInstance1.AddSubSystem(gain_2_para);
	subsystem_handle Sum_1 = SimInstance1.AddSubSystem(sum_1_para);
	simulationcontrol::SIMCONNECTION Connection_LTI_1, Connection_Gain_1, Connection_Sum_1, Connection_Gain_2;

	Connection_LTI_1.resize(1, 2);
	Connection_LTI_1(0, simulationcontrol::subsystemID) = Gain_1.ID;
	Connection_LTI_1(0, simulationcontrol::outputportID) = 0;

	Connection_Sum_1.resize(2, 2);
	Connection_Sum_1(0, simulationcontrol::subsystemID) = sinewave_1.ID;
	Connection_Sum_1(0, simulationcontrol::outputportID) = 0;

	Connection_Sum_1(1, simulationcontrol::subsystemID) = Gain_2.ID;
	Connection_Sum_1(1, simulationcontrol::outputportID) = 0;

	Connection_Gain_1.resize(1, 2);
	Connection_Gain_1(0, simulationcontrol::subsystemID)  = Sum_1.ID;
	Connection_Gain_1(0, simulationcontrol::outputportID) = 0;

	Connection_Gain_2.resize(1, 2);
	Connection_Gain_2(0, simulationcontrol::subsystemID) = LTI_1.ID; 
	Connection_Gain_2(0, simulationcontrol::outputportID) = 3;

	SimInstance1.MakeConnection(LTI_1.ID, Connection_LTI_1);
	SimInstance1.MakeConnection(Gain_1.ID, Connection_Gain_1);
	SimInstance1.MakeConnection(Sum_1.ID, Connection_Sum_1);
	SimInstance1.MakeConnection(Gain_2.ID, Connection_Gain_2);

	SimInstance1.DefineDataLogging(LTI_1.ID, 0, "v");
	SimInstance1.DefineDataLogging(LTI_1.ID, 1, "alpha");
	SimInstance1.DefineDataLogging(LTI_1.ID, 2, "theta");
	SimInstance1.DefineDataLogging(LTI_1.ID, 3, "q");

	bool flag = SimInstance1.PreRunProcess();
	MatlabIO Recorder;
	int N_steps = 500;

	MatrixXd matdata;
	matdata.resize(N_steps + 1, 7); // TIME 0 input 1  v 2 alpha 3 theta 4 q 5 sum 6
	matdata.setZero();// reset the buffer to zero

	if (flag) { // if successful, run updates

		VectorXd extern_input;
		SimInstance1.ReshapeExternalInputVector(extern_input);
		for (int i = 0; i < N_steps; i++)
		{
			matdata(i, 0) = SimInstance1.Run_GetSystemTime();
			// save v alpha theta q
			matdata(i, 1) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(0);
			matdata(i, 2) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(0);
			matdata(i, 3) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(1);
			matdata(i, 4) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(2);
			matdata(i, 5) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(3);
			matdata(i, 6) = SimInstance1.Run_GetSubsystemOuput(Sum_1.ID)(0);
			//std::cout << "Sum block output: " << SimInstance1.Run_GetSubsystemOuput(Sum_1.ID)(0) << " \n ";
			SimInstance1.Run_Update(extern_input);
		}
		matdata(N_steps, 0) = SimInstance1.Run_GetSystemTime();
		matdata(N_steps, 1) = SimInstance1.Run_GetSubsystemOuput(sinewave_1.ID)(0);
		matdata(N_steps, 2) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(0);
		matdata(N_steps, 3) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(1);
		matdata(N_steps, 4) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(2);
		matdata(N_steps, 5) = SimInstance1.Run_GetSubsystemOuput(LTI_1.ID)(3);
		matdata(N_steps, 6) = SimInstance1.Run_GetSubsystemOuput(Sum_1.ID)(0);
		if (Recorder.SaveToMatFile(matdata, "linear_system_state.mat", "Data")) {
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

#ifdef DEBUG_ENGINE
	FADEC::EngineLogic logic1(FADEC::LOGIC_SHUTDOWN);
	FADEC::EngineLogicInput logic1_input;
	FADEC::EngineLogicOutput logic1_output;

	// parameter list
	double N2_damping_ = 1.0 / 12.0;
	double N2_initial = 0.0;
	double Kp = 0.5;
	double Ki = 0.25*pow((N2_damping_ + Kp) ,2);

	double norm_up_vel = 0.2742 / 2.45;
	double norm_down_vel = 0.2742 / 6.5;
	double start_up_vel = 3.7*0.179 / 20.0;


	double c_norm_upper = N2_damping_ * norm_up_vel / Ki;
	double c_norm_lower = -N2_damping_ * norm_down_vel / Ki;

	double c_start_upper = N2_damping_ * start_up_vel / Ki;
	double c_start_lower = -N2_damping_ * start_up_vel / Ki;

	double N2_injection = 0.316;
	double N2_starter_cutoff = 0.5;
	double N2_starter_torque = N2_damping_ * N2_injection;
	double N2_starter_rate = -N2_starter_torque / (N2_starter_cutoff - N2_injection);
	double N2_starter_intercept = N2_starter_torque - N2_injection * N2_starter_rate;


	// n2 dynamics

	std::vector< subsystem_handle*> handle_pointer_list;

	linearsystem::IntegratorParameter N2_dynamics;
	N2_dynamics.num_of_channels = 1;
	linearsystem::IntegratorInitialCondition N2_initialcondition;
	N2_initialcondition.X_0.resize(1);
	N2_initialcondition.X_0(0) = N2_initial;
	subsystem_handle N2rotordynamics = SimInstance1.AddSubSystem(N2_dynamics, N2_initialcondition);

	handle_pointer_list.push_back(&N2rotordynamics);

	mathblocks::GainParameter N2_damping_param_;
	N2_damping_param_.K.resize(1,1);
	N2_damping_param_.K(0) = N2_damping_;
	N2_damping_param_.Mode = mathblocks::ElementWise;
	N2_damping_param_.num_of_inputs = 1;
	subsystem_handle N2_damping = SimInstance1.AddSubSystem(N2_damping_param_);

	handle_pointer_list.push_back(&N2_damping);

	mathblocks::SumParameter N2_dynamics_sum_param_;
	N2_dynamics_sum_param_.input_dimensions = 1;
	N2_dynamics_sum_param_.num_of_inputs = 2;
	N2_dynamics_sum_param_.sign_list.resize(2);
	N2_dynamics_sum_param_.sign_list(0) = 1.0*mathblocks::SUM_POSITIVE;
	N2_dynamics_sum_param_.sign_list(1) = 1.0*mathblocks::SUM_NEGATIVE;
	subsystem_handle N2_dynamics_sum = SimInstance1.AddSubSystem(N2_dynamics_sum_param_);

	handle_pointer_list.push_back(&N2_dynamics_sum);

	SimInstance1.EditConnectionMatrix(N2rotordynamics, 0, N2_dynamics_sum.ID, 0);
	SimInstance1.EditConnectionMatrix(N2_damping, 0, N2rotordynamics.ID, 0);
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum, 1, N2_damping.ID, 0);

	// inside n2 speed control 
	linearsystem::PIDcontrollerParameter N2controller_param_;
	N2controller_param_.integration_control_on = true;
	N2controller_param_.Kp =  Kp;
	N2controller_param_.Ki =  Ki;
	N2controller_param_.Kd = 0.0;
	N2controller_param_.num_of_channels = 1;
	N2controller_param_.Tf = 1.0 / 100.0;
	subsystem_handle PID = SimInstance1.AddSubSystem(N2controller_param_);

	handle_pointer_list.push_back(&PID);

	// 2 satruation part 
	mathblocks::SumParameter N2controller_sum_1_param_;
	N2controller_sum_1_param_.input_dimensions = 1;
	N2controller_sum_1_param_.num_of_inputs = 2;
	N2controller_sum_1_param_.sign_list.resize(2);
	N2controller_sum_1_param_.sign_list(0) = 1.0*mathblocks::SUM_POSITIVE;
	N2controller_sum_1_param_.sign_list(1) = 1.0*mathblocks::SUM_NEGATIVE;
	subsystem_handle N2_dynamics_sum_1 = SimInstance1.AddSubSystem(N2controller_sum_1_param_);

	handle_pointer_list.push_back(&N2_dynamics_sum_1);
	
	discontinuoussystem::SwitchParameter N2controller_switch_1_param_;
	N2controller_switch_1_param_.num_of_channels = 1;
	N2controller_switch_1_param_.switch_value = 0.6;
	subsystem_handle N2controller_switch_1 = SimInstance1.AddSubSystem(N2controller_switch_1_param_);

	handle_pointer_list.push_back(&N2controller_switch_1);

	discontinuoussystem::SaturationParameter N2controller_saturation_norm_param_;
	N2controller_saturation_norm_param_.lower_bound = c_norm_lower;
	N2controller_saturation_norm_param_.upper_bound = c_norm_upper;
	N2controller_saturation_norm_param_.num_of_channels = 1;
	N2controller_saturation_norm_param_.type = discontinuoussystem::SATURATION_BOTH;
	subsystem_handle N2controller_saturation_norm = SimInstance1.AddSubSystem(N2controller_saturation_norm_param_);

	handle_pointer_list.push_back(&N2controller_saturation_norm);

	discontinuoussystem::SaturationParameter N2controller_saturation_start_param_;
	N2controller_saturation_start_param_.lower_bound = c_start_lower;
	N2controller_saturation_start_param_.upper_bound = c_start_upper;
	N2controller_saturation_start_param_.num_of_channels = 1;
	N2controller_saturation_start_param_.type = discontinuoussystem::SATURATION_BOTH;
	subsystem_handle N2controller_saturation_start = SimInstance1.AddSubSystem(N2controller_saturation_start_param_);

	handle_pointer_list.push_back(&N2controller_saturation_start);

	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 0, simulationcontrol::external, 0); // 0 for N2_cmd
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 1, N2rotordynamics.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_switch_1, 0, N2rotordynamics.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_saturation_norm, 0, N2_dynamics_sum_1.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_saturation_start, 0, N2_dynamics_sum_1.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_switch_1, 1, N2controller_saturation_norm.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_switch_1, 2, N2controller_saturation_start.ID, 0);
	SimInstance1.EditConnectionMatrix(PID, 1, N2controller_switch_1.ID, 0);
	SimInstance1.EditConnectionMatrix(PID, 0, simulationcontrol::external, 0); // 1 for FF_flow

	// inside FADEC
	mathblocks::SumParameter N2controller_sum_2_param_; // for starter torque
	N2controller_sum_2_param_.input_dimensions = 1;
	N2controller_sum_2_param_.num_of_inputs = 2;
	N2controller_sum_2_param_.sign_list.resize(2);
	N2controller_sum_2_param_.sign_list(0) = 1.0*mathblocks::SUM_POSITIVE;
	N2controller_sum_2_param_.sign_list(1) = 1.0*mathblocks::SUM_POSITIVE;
	subsystem_handle  N2controller_sum_2 = SimInstance1.AddSubSystem(N2controller_sum_2_param_);

	handle_pointer_list.push_back(&N2controller_sum_2);

	mathblocks::SumParameter N2controller_sum_3_param_;// starter torque + PID controller
	N2controller_sum_3_param_.input_dimensions = 1;
	N2controller_sum_3_param_.num_of_inputs = 2;
	N2controller_sum_3_param_.sign_list.resize(2);
	N2controller_sum_3_param_.sign_list(0) = 1.0*mathblocks::SUM_POSITIVE;
	N2controller_sum_3_param_.sign_list(1) = 1.0*mathblocks::SUM_POSITIVE;
	subsystem_handle  N2controller_sum_3 = SimInstance1.AddSubSystem(N2controller_sum_3_param_);

	handle_pointer_list.push_back(&N2controller_sum_3);

	mathblocks::GainParameter N2controller_N2starterrate_param_; // N
	N2controller_N2starterrate_param_.K.resize(1, 1);
	N2controller_N2starterrate_param_.K(0, 0) = N2_starter_rate;
	N2controller_N2starterrate_param_.Mode = mathblocks::ElementWise;
	N2controller_N2starterrate_param_.num_of_inputs = 1;
	subsystem_handle  N2controller_N2starterrate = SimInstance1.AddSubSystem(N2controller_N2starterrate_param_);

	handle_pointer_list.push_back(&N2controller_N2starterrate);

	mathblocks::ConstantParameter N2controller_N2starterintercept_param_;
	N2controller_N2starterintercept_param_.value.resize(1);
	N2controller_N2starterintercept_param_.value(0) = N2_starter_intercept;
	subsystem_handle  N2controller_N2starterintercept = SimInstance1.AddSubSystem(N2controller_N2starterintercept_param_);

	handle_pointer_list.push_back(&N2controller_N2starterintercept);

	discontinuoussystem::SaturationParameter N2controller_starterlimiter_param_;
	N2controller_starterlimiter_param_.lower_bound = 0.0;
	N2controller_starterlimiter_param_.upper_bound = N2_starter_torque;
	N2controller_starterlimiter_param_.num_of_channels = 1;
	N2controller_starterlimiter_param_.type = discontinuoussystem::SATURATION_BOTH;
	subsystem_handle  N2controller_starterlimiter = SimInstance1.AddSubSystem(N2controller_starterlimiter_param_);

	handle_pointer_list.push_back(&N2controller_starterlimiter);

	mathblocks::MultiplicationParam  N2controller_product_param_;
	N2controller_product_param_.input1_dimension(0) = 1;
	N2controller_product_param_.input1_dimension(1) = 1;
	N2controller_product_param_.input2_dimension(0) = 1;
	N2controller_product_param_.input2_dimension(1) = 1;
	N2controller_product_param_.Mode = mathblocks::ElementWise;
	subsystem_handle N2controller_product = SimInstance1.AddSubSystem(N2controller_product_param_);

	handle_pointer_list.push_back(&N2controller_product);

	SimInstance1.EditConnectionMatrix(N2controller_N2starterrate, 0, N2rotordynamics.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_sum_2, 0, N2controller_N2starterrate.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_sum_2, 1, N2controller_N2starterintercept.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_starterlimiter, 0, N2controller_sum_2.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_product, 0, simulationcontrol::external, 0); // 2 for starter
	SimInstance1.EditConnectionMatrix(N2controller_product, 1, N2controller_starterlimiter.ID, 0);

	SimInstance1.EditConnectionMatrix(N2controller_sum_3, 0, N2controller_product.ID, 0);
	SimInstance1.EditConnectionMatrix(N2controller_sum_3, 1, PID.ID, 0);

	SimInstance1.EditConnectionMatrix(N2_dynamics_sum, 0, N2controller_sum_3.ID, 0);


	// make connection

	int num_of_systems_defined = handle_pointer_list.size();

	for (int i = 0; i < num_of_systems_defined; i++) {
		SimInstance1.MakeConnection(*handle_pointer_list[i]);
	}

	// define the data log tag

	//SimInstance1.DefineDataLogging(N2rotordynamics.ID, 0, "N2_sim");
	//SimInstance1.DefineDataLogging(N2controller_starterlimiter.ID, 0, "starter_torque_sim");
	//SimInstance1.DefineDataLogging(PID.ID, 0, "PID_sim");
	//SimInstance1.DefineDataLogging(N2controller_switch_1.ID, 0, "switch_sim");
	//SimInstance1.DefineDataLogging(N2_dynamics_sum_1.ID, 0, "N2_dynamics_sum_1_sim");

	bool flag = SimInstance1.PreRunProcess();
	int N_steps = 6500;
	logic1_input.Ignition = true;
	logic1_input.Starter = true;
	double throttle = 0.588;


	if (flag) { // if successful, run updates

		VectorXd extern_input;
		SimInstance1.ReshapeExternalInputVector(extern_input);
		t = clock();
		for (int i = 0; i < N_steps; i++)
		{
			logic1_input.N2 = SimInstance1.Run_GetSubsystemOuput(N2rotordynamics.ID)(0);
			if (SimInstance1.Run_GetSystemTime() > 70) {
				if (SimInstance1.Run_GetSystemTime() > 80) {
					if (SimInstance1.Run_GetSystemTime() > 100) {
						throttle = 0.588;
					}
					else {
						throttle = 0.28 + 0.588 + 0.114;
					}
				}
				else {
					throttle = 0.114 + 0.588;
				}
			}

			logic1_output = logic1.UpdateLogic(logic1_input);
			extern_input(0) = logic1_output.FuelFlow;
			extern_input(1) = throttle;
			extern_input(2) = logic1_output.startertorque;
			//std::cout << "Sum block output: " << SimInstance1.Run_GetSubsystemOuput(Sum_1.ID)(0) << " \n ";
			SimInstance1.Run_Update(extern_input);
		}
		t = clock() - t;
		printf("Calculation Takes %d clicks (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
	}
	else {
		std::cout << "Initialization failed, check subsystem connections" << std::endl;
	}
#endif // DEBUG_ENGINE

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
