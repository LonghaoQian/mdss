#include "pch.h"
#include "AeroAngle.h"


aero::AeroAngle::AeroAngle(const AeroAngleParameter& parameter)
{
}


void aero::AeroAngle::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
{
	output(0) = input
}

aero::AeroAngle::~AeroAngle()
{
}
