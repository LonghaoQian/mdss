#include "stdafx.h"
#include "Subsystem.h"




Subsystem::Subsystem()
{
}


Subsystem::~Subsystem()
{
}

void Subsystem::SetInputConnection(const MatrixX2i& connection)
{
	system_info.input_connection = connection;
}

subsystem_info Subsystem::GetSystemInfo()
{
	return system_info;
}


