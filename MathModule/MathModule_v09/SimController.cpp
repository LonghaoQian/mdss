#include "stdafx.h"
#include "SimController.h"


SimController::SimController(const SolverConfig& config, const vector<subsystem_info>& system_construct, const vector<VectorXd>& parameter_list)
{
	system_ok = false;
	solver_config = config;
	num_of_subsystems = system_construct.size();
	for (int i = 0; i < num_of_subsystems;i++)
	{
		Add_subsystem(system_construct[i].subsystem_type, input_connection[i], parameter_list[i]); // input all system info to the sim control object
	}
	PreRunProcess();
}


int SimController::Add_subsystem(unsigned  int system_type, const MatrixX2i& input_connection, const VectorXd& parameters)
{
	/* Logic:
	1. push a pointer into subsystem_list
	2. load parameters
	3. load initial conditions
	4. check the connectivity
	5. update the connnectivity matrix
	*/
	int flag = 0;
	input_connection.emplace_back(

	switch (system_type)
	{
	case LTI:
		subsystem_list.emplace_back(new LTIsystem);// push the 
		break;
	case RIGIDBODY:
		break;
	default: 
		flag = 1;// no specified class

	}

	return flag;
}

int SimController::PreRunProcess()
{
	/* Logic:
	1. Assign input-output connection to each subsystem
	2. Record the external outputs
	3. Check the validity of connections: a) whether more than one outputs are connected to the same input port, b) whether a port is left with nothing as input
	4. Update the connectivity matrix
	5. Check the total number of loops: a) check algebraic loops (whether a loop only contains subsystems with no continous states)
	6. Update the connectivity matrix
	*/
	// step 1 Assign input-output connection to each subsystem

	// step 4 the connectivity matrix
	connectivity.resize(num_of_subsystems, num_of_subsystems);
	for (int i = 0; i < num_of_subsystems; i++)
	{

	}
	return 0;
}

SimController::~SimController()
{
}
