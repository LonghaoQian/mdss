#include "pch.h"
#include "Math_Gain.h"


Math_Gain::Math_Gain(const Gainparameter& _gainparameter)
{
	system_info.system_type = Gain;
	system_info.DIRECT_FEED_THROUGH = true;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	K = _gainparameter.K;
	system_info.system_parameter_ok = 0;
	ready_to_run = true;
	Gain_Type = _gainparameter.type;
	switch (_gainparameter.type)
	{
	case GAIN_MATRIX:
		system_info.num_of_inputs = _gainparameter.K.cols();
		system_info.num_of_outputs = _gainparameter.K.rows();
		break;
	case GAIN_ElEMENT_WISE:
		system_info.num_of_inputs = _gainparameter.K.rows();
		system_info.num_of_outputs = system_info.num_of_inputs;
		break;
	case GAIN_SCALER:
		system_info.num_of_inputs = _gainparameter.num_of_inputs;
		system_info.num_of_outputs = _gainparameter.num_of_inputs;
		break;
	default:
		system_info.num_of_inputs = _gainparameter.K.cols();
		system_info.num_of_outputs = _gainparameter.K.rows();
		break;
	}
	output.resize(system_info.num_of_outputs);
	output.setZero(system_info.num_of_outputs);
	ready_to_run = true;
}

void Math_Gain::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	
	switch (Gain_Type)
	{
	case GAIN_MATRIX:
		output = K * input;
		break;
	case GAIN_ElEMENT_WISE:
		output.resize(system_info.num_of_outputs);
		for (int i = 0; i < system_info.num_of_outputs; i++)
		{
			output(i) = input(i)*K(i,0);
		}
		break;
	case GAIN_SCALER:
		output = K(0,0) * input;
		break;
	default:
		output = K * input;
		break;
	}
}

void Math_Gain::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & temp_derivative)
{
}

void Math_Gain::IncrementState()
{
}

VectorXd Math_Gain::GetState()
{
	return VectorXd();
}

void Math_Gain::DisplayParameters()
{
	cout << "Gain is :" << endl;
	cout << "K = " << endl << K << endl;
}
void Math_Gain::DisplayInitialCondition()
{
	cout<< "------No initial condition for gain block----------" << endl;
}


Math_Gain::~Math_Gain()
{
}
