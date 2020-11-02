#include "pch.h"
#include "Aircraftmodel.h"
namespace  aircraft {
	AircraftDynamicModel::AircraftDynamicModel(const modelparameter & param, const initialcondition & IC)
	{
	}

	AircraftDynamicModel::~AircraftDynamicModel()
	{
	}

	void AircraftDynamicModel::UpdateSimulation(const C172input & input)
	{
	}

	void AircraftDynamicModel::EndSimulation()
	{
	}

	bool AircraftDynamicModel::ResetParameter(const modelparameter & param)
	{
		return false;
	}

	bool AircraftDynamicModel::ResetSimulation(const initialcondition & IC)
	{
		return false;
	}
}