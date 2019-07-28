#include "stdafx.h"
#include "Source_SignalGenerator.h"


Source_SignalGenerator::Source_SignalGenerator()
{
	system_info.DIRECT_FEED_THROUGH = false;
	system_info.NO_CONTINUOUS_STATE = true;
	system_info.num_of_continuous_states = 0;
	system_info.num_of_inputs = 0;
}


Source_SignalGenerator::~Source_SignalGenerator()
{
}
