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

	system_info.num_of_inputs = 26;
	system_info.num_of_outputs = 11;

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
	// step. 1 calculate the wind speed in inertial frame
	WindSpeed(mathauxiliary::VECTOR_X) = -cos(input(AERO_INPUT_WINDDIRECTION)) * input(AERO_INPUT_HORIZONTALWINDSPEED);
	WindSpeed(mathauxiliary::VECTOR_Y) = -sin(input(AERO_INPUT_WINDDIRECTION)) * input(AERO_INPUT_HORIZONTALWINDSPEED);
	WindSpeed(mathauxiliary::VECTOR_Z) = -input(AERO_INPUT_VERTICALWINDSPEED);
	// transform the windspeed to the body-fixed frame
	mathauxiliary::ConvertVectorToRotationMatrix(input.segment(AERO_INPUT_R_BI00,9), R_BI);
	WindSpeedBody = R_BI * WindSpeed;
	// step. 2 calculate the true airspeed
	TASBody = input.segment(AERO_INPUT_Vbx, 3) - WindSpeedBody; // the airspeed in the body-fixed frame  = [U v w]^T
	TAS = TASBody.norm();
	output(AERO_OUTPUT_TAS) = TAS;
	// if the airspeed is below a threshold, then saturate the tas to avoid singularity in the subsequent calculation
	if (TAS < param_.min_airspeed_) { 
		TAS_cal = param_.min_airspeed_;
	}
	else {
		TAS_cal = TAS;
	}
	// step. 3 mach number
	output(AERO_OUTPUT_MACHNUMBER) = output(AERO_OUTPUT_TAS) / input(AERO_INPUT_SOUNDSPEED);
	// step. 4 other aero angles
	/* alpha  = atan2(w,U)
	   beta = asin(v/TAS)
	*/
	// angle of attack
	output(AERO_OUTPUT_AOA) = atan2(TASBody(mathauxiliary::VECTOR_Z), TASBody(mathauxiliary::VECTOR_X));
	// side slip anlge
	output(AERO_OUTPUT_SIDESLIP) = asin(TASBody(mathauxiliary::VECTOR_Y)/ TAS_cal); 
	// dynamics pressure
	output(AERO_OUTPUT_DYNAMICPRESSURE) = 0.5* input(AERO_INPUT_RHO) * TAS * TAS; 
	// angular rate normalizer cbar/(2V), b/(2V)
	lon_normalizer = param_.c_bar_ / (2 * TAS_cal);
	lat_normalizer = param_.b_ / (2 * TAS_cal);
 	/* 
	   TAS_dot = Vb^T Vb_dot / TAS ( assume the rate of change of the wind speed is negligible) will modify this later
	*/
	TAS_dot = TASBody.dot(input.segment(AERO_INPUT_Vbdotx,3)) / TAS_cal;
	TempCal_1 = TASBody(mathauxiliary::VECTOR_X)*TASBody(mathauxiliary::VECTOR_X) + TASBody(mathauxiliary::VECTOR_Z) * TASBody(mathauxiliary::VECTOR_Z);

	if (TempCal_1 < param_.min_airspeed_) { // if the velocity is lower than a threshold, then use the theshold as the velocity
		TempCal_1 = param_.min_airspeed_;
	}
	// step. 5 angular rates
	// AOA rate
	output(AERO_OUTPUT_AOARATE) = lon_normalizer * (input(AERO_INPUT_Vbdotz)*TASBody(mathauxiliary::VECTOR_X) - input(AERO_INPUT_Vbdotx)*TASBody(mathauxiliary::VECTOR_Z)) / TempCal_1;
	// beta rate
	TempCal_2 = TASBody(mathauxiliary::VECTOR_Y) / TAS_cal;
	output(AERO_OUTPUT_SIDESLIPRATE) = lat_normalizer * (output(AERO_OUTPUT_TAS)*input(AERO_INPUT_Vbdoty)- TASBody(mathauxiliary::VECTOR_Y)*TAS_dot) / (TAS_cal*TAS_cal*sqrt(1- TempCal_2* TempCal_2));
	// normalized angular rate:
	output(AERO_OUTPUT_Pbar) = lat_normalizer * input(AERO_INPUT_P);
	output(AERO_OUTPUT_Qbar) = lon_normalizer * input(AERO_INPUT_Q);
	output(AERO_OUTPUT_Rbar) = lat_normalizer * input(AERO_INPUT_R);
	// step. 6 flight path angle
	if (TAS < 0.1*param_.min_airspeed_) {
		output(AERO_OUTPUT_GAMMA) = 0.0; // if the airspeed is too low, stop calculation
	} else {
		output(AERO_OUTPUT_GAMMA) = asin(-input(AERO_INPUT_VIz) / input.segment(AERO_INPUT_VIx, 3).norm());
	}
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
	std::cout << "------No initial condition for aero angle block----------" << std::endl;
}

aero::AeroAngle::~AeroAngle()
{
}


aero::CAS::CAS(const CASConversionParameter & parameter)
{

}

aero::CAS::~CAS()
{
}

void aero::CAS::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
{
	// no differential equations for CAS block
}

void aero::CAS::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	/*
	// step. 6 calibrated airspeed
	// calculate the impact pressure based on Isentropic flow assumption https://en.wikipedia.org/wiki/Impact_pressure
	qc = input(AERO_INPUT_STATICPRESSURE)*(pow((1.0 + 0.2*output(AERO_OUTPUT_MACHNUMBER)*output(AERO_OUTPUT_MACHNUMBER)), 3.5) - 1.0);
	// then calculate the calibrated airspeed https://en.wikipedia.org/wiki/Calibrated_airspeed
	if (output(AERO_OUTPUT_MACHNUMBER) > 1.0) {
		// supersonic speed
		// output(AERO_OUTPUT_CAS) = a0 * pow(())
	} else {
		// subsonic speed
		output(AERO_OUTPUT_CAS) = a0 * sqrt( 5.0 * ( pow( (qc/P0 + 1.0), 0.28571428571) - 1.0) ); // 2.0/7.0 = 0.28571428571
	}


*/
}

void aero::CAS::IncrementState()
{
	// No increment state for CAS conversion block
}

void aero::CAS::DisplayParameters()
{

}

void aero::CAS::DisplayInitialCondition()
{
	std::cout << "------No initial condition for CAS block----------" << std::endl;
}
