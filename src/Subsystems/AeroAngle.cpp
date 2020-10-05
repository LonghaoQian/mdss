#include "pch.h"
#include "AeroAngle.h"



aero::AeroAngle::AeroAngle(const AeroAngleParameter& parameter)
{
	param_ = parameter;
	system_info.type = aero_AROANGLE;
	system_info.category = AERODYNAMICS;

	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;

	system_info.num_of_continuous_states = 0;

	system_info.num_of_inputs = 11;
	system_info.num_of_outputs = 10;

	system_info.system_parameter_ok = 0;
	ready_to_run = true;

	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);
}

void aero::AeroAngle::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// NO differential euqation for direct feed through 
}

void aero::AeroAngle::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	/*
	Aerodynamics angle calcuations:

	input : 
	0 rho
	1 soundspeed
	2 Vbx
	3 Vby
	4 Vbz
	5 p
	6 q
	7 r
	8 Ab_x // Find veloity derivative in body fixed frame :Vb_dot=Ab-wXVb
	9 Ab_y // 
	10 Ab_z // 


	//TO DO
	11 - 19 R_BI
	20 Vw_x
	21 Vw_y
	22 Vw_z

	output : 
	0 TAS
	1 MachNumber
	2 alpha
	3 beta
	4 dynamic pressure
	5 alpha_dot_bar
	6 beta_dot_bar
	7 p_bar
	8 q_bar
	9 r_bar
	*/
	TAS = input.segment(AERO_INPUT_Vbx, 3).norm();
	output(AERO_OUTPUT_TAS) = TAS;
	if (TAS < param_.min_airspeed_) { // saturate the tas for calculation
		TAS_cal = param_.min_airspeed_;
	}
	else {
		TAS_cal = TAS;
	}
	output(AERO_OUTPUT_MACHNUMBER) = output(AERO_OUTPUT_TAS) / input(AERO_INPUT_SOUNDSPEED);      // mach number
	/* alpha  = atan2(w,U)
	   beta = asin(v/TAS)
	*/
	// angle of attack
	output(AERO_OUTPUT_AOA) = atan2(input(AERO_INPUT_Vbz), input(AERO_INPUT_Vbx));  
	// side slip anlge
	output(AERO_OUTPUT_SIDESLIP) = asin(input(AERO_INPUT_Vby) / TAS_cal); 
	// dynamics pressure
	output(AERO_OUTPUT_DYNAMICPRESSURE) = 0.5* input(AERO_INPUT_RHO) * TAS * TAS; 
	// angular rate normalizer cbar/(2V), b/(2V)
	lon_normalizer = param_.c_bar_ / (2 * TAS_cal);
	lat_normalizer = param_.b_ / (2 * TAS_cal);
 	/* 
	   TAS_dot = Vb^T Vb_dot / TAS
	*/
	TAS_dot = input.segment(AERO_INPUT_Vbx,3).dot(input.segment(AERO_INPUT_Vbdotx,3)) / TAS_cal;
	TempCal_1 = input(AERO_INPUT_Vbx)*input(AERO_INPUT_Vbx) + input(AERO_INPUT_Vbz) * input(AERO_INPUT_Vbz);

	if (TempCal_1 < param_.min_airspeed_) { // if the velocity is lower than a threshold, then use the theshold as the velocity
		TempCal_1 = param_.min_airspeed_;
	}

	// AOA rate
	output(AERO_OUTPUT_AOARATE) = lon_normalizer * (input(AERO_INPUT_Vbdotz)*input(AERO_INPUT_Vbx)- input(AERO_INPUT_Vbdotx)*input(AERO_INPUT_Vbz)) / TempCal_1;
	// beta rate
	TempCal_2 = input(AERO_INPUT_Vby) / TAS_cal;
	output(AERO_OUTPUT_SIDESLIPRATE) = lat_normalizer * (output(AERO_OUTPUT_TAS)*input(AERO_INPUT_Vbdoty)-input(AERO_INPUT_Vby)*TAS_dot) / (TAS_cal*TAS_cal*sqrt(1- TempCal_2* TempCal_2));
	// normalized angular rate:
	output(AERO_OUTPUT_Pbar) = lat_normalizer * input(AERO_INPUT_P);
	output(AERO_OUTPUT_Qbar) = lon_normalizer * input(AERO_INPUT_Q);
	output(AERO_OUTPUT_Rbar) = lat_normalizer * input(AERO_INPUT_R);

	// TO DO: add external wind


}

void aero::AeroAngle::IncrementState()
{
	// No increment state for aero angle block
}

void aero::AeroAngle::DisplayParameters()
{
	std::cout << "---------------------" << std::endl;
	std::cout << "Minimum TAS is set to : " << param_.min_airspeed_  << " m/s "<< std::endl;
	std::cout << "Wing span is: " << param_.b_ << " m " << std::endl;
	std::cout << "Mean chord is : " << param_.c_bar_ << " m " << std::endl;
}

void aero::AeroAngle::DisplayInitialCondition()
{
	std::cout << "------No initial condition for atmopshere block----------" << std::endl;
}

aero::AeroAngle::~AeroAngle()
{
}
