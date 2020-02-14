#include "pch.h"
#include "AeroAngle.h"



aero::AeroAngle::AeroAngle(const AeroAngleParameter& parameter)
{
	param_ = parameter;
	system_info.system_type = AROANGLE;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 22;
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
	output(0) = input.segment(2, 3).norm();
	mathauxiliary::SaturationElementalWise(output(0), 10000000.0, param_.min_airspeed_); // TAS 
	output(1) = output(0) / input(1); // mach number
	output(2) = atan2(input(4), input(2));
	output(3) = asin(input(3) / output(0));
	output(4) = 0.5* input(0) * output(0) * output(0);
	double lon_normalizer = param_.c_bar_ / (2 * output(0));
	double lat_normalizer = param_.b_ / (2 * output(0));
 	/* alpha  = atan2(w,U)
	 beta = asin(v/TAS)  
	 TAS_dot = Vb^T Ab / TAS
	*/
	omega_b(0) = input(5);
	omega_b(1) = input(6);
	omega_b(2) = input(7);
	Vb = input.segment(2, 3);
	Vb_dot = input.segment(8, 3);
	double TAS_dot = Vb.dot(Vb_dot) / output(0);

	output(5) = lon_normalizer * (Vb_dot(3)*input(2)- Vb_dot(0)*input(4)) / (input(2)* input(2) + input(4) * input(4));
	output(6) = lat_normalizer * (output(0)*Vb_dot(1)-input(3)*TAS_dot) / (output(0)*output(0)*sqrt(1-(input(3)/ output(0))*(input(3)/ output(0))));
	output(7) = lat_normalizer * input(5);
	output(8) = lon_normalizer * input(6);
	output(9) = lat_normalizer * input(7);
}

void aero::AeroAngle::IncrementState()
{

}

void aero::AeroAngle::DisplayParameters()
{
	std::cout << "Minimum TAS is set to : " << param_.min_airspeed_  << " m/s "<< std::endl;
}

void aero::AeroAngle::DisplayInitialCondition()
{
	std::cout << "------No initial condition for atmopshere block----------" << std::endl;
}

aero::AeroAngle::~AeroAngle()
{
}
