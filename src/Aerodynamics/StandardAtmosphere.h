#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"
#include "MatlabIO.h"
struct StandardAtmosphereParameter {
	const char* atmoshpere_name_;
};
class StandardAtmosphere :
	public Subsystem
{
private:
	// parameters
	MatlabIO atmosphere_file;
	MatrixXd atmosphere_data_;
	mathauxiliary::Lookup_1D atmopshere_table_;
	bool load_flag_;
	/*TO DO: built-in atmosphere data */
public:
	StandardAtmosphere(const StandardAtmosphereParameter& parameter);
	void DifferentialEquation(const double& t,
							  const VectorXd& state,
							  const VectorXd& input, 
							  VectorXd& derivative);
	void OutputEquation(const double& t,
						const VectorXd& state, 
						const VectorXd& input, VectorXd& output);
	void IncrementState();
	void DisplayParameters();
	void DisplayInitialCondition();
	~StandardAtmosphere();
};

