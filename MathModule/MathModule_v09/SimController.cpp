#include "stdafx.h"
#include "SimController.h"


SimController::SimController(const SolverConfig& config)
{
	solver_config = config;// load the solver information
}

bool SimController::AddSubSystem(const LTIParameter& parameters, const  LTIInitialCondition& IC)
{
	/* Logic:
	1. push a pointer into subsystem_list
	2. load parameters
	3. load initial conditions
	4. check the connectivity
	5. update the connnectivity matrix
	*/
	int flag;
	subsystem_info system_info;
	subsystem_list.emplace_back(new LTIsystem(parameters, IC));
	system_info = subsystem_list.back()->GetSystemInfo();
	// update number of subsystems
	num_of_subsystems = subsystem_list.size();
	if (system_info.system_parameter_ok == true)
	{
		flag = true;
	}
	else {
		flag = false;
	}
	return flag;
}

bool SimController::AddSubSystem(const RigidBodyParameter & parameters, const RigidBodyCondition & IC)
{
	bool flag;
	subsystem_info system_info;
	subsystem_list.emplace_back(new RigidBody(parameters, IC));
	system_info = subsystem_list.back()->GetSystemInfo();
	// update number of subsystems
	num_of_subsystems = subsystem_list.size();
	if (system_info.system_parameter_ok == true)
	{
		flag = true;
	}
	else {
		flag = false;
	}
	return flag;
}

bool SimController::MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping)
{
	// parse the connection mapping 
	// the connection mapping is defined in a Xx3 matrix
	// index of the input port, system ID of the output, index of the output 
	bool flag = false;
	if (system_ID > num_of_subsystems)
	{
		cout << "CONNECTION ERROR: The system ID goes out of bound, the number of subsystem is " << num_of_subsystems << endl;
		cout << " while the input system ID is (system_ID<number of subsystem)" << system_ID << endl;
	}
	else {
		if(connection_mapping.rows()== subsystem_list[system_ID]->GetSystemInfo().num_of_inputs)
		{
			subsystem_list[system_ID]->SetInputConnection(connection_mapping);
			flag = true;
		   }
		else {
			cout << "CONNECTION ERROR: The connection mapping matrix does not match the number of inputs of system # " << system_ID << endl;
		}

	}
	
	return flag;
}

int SimController::Run()
{
	/*Logic
	1. update the external input to the external_input butter

	2. 

	update all subsystem states based on the increment
	
	*/


	return 0;
}

int SimController::PostRunProcess()
{
	return 0;
}

void SimController::DisplayTopology()
{
	cout <<"The total Number of Subsystems is "<< num_of_subsystems << endl;
	subsystem_info system_info;
	for (int i = 0;i< num_of_subsystems; i++)
	{
		cout << "+++++++++++++++++++++++++++++++++++++++++++" << endl;
		system_info = subsystem_list[i]->GetSystemInfo();
		cout << " Subsystem # " << i << " is of type " << system_info.system_type << endl;
	// display system parameters
		subsystem_list[i]->DisplayParameters();
	// display system initial condition
		subsystem_list[i]->DisplayInitialCondition();
	// display system input connection
		for (int j = 0; j < subsystem_list[i]->GetSystemInfo().num_of_inputs; j++)
		{
			if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0) >= 0)
			{
				
				cout << "input port # " << j << " is connected to port # " << 
					subsystem_list[i]->GetSystemInfo().input_connection(j, 1) << 
					" of subsystem # " << subsystem_list[i]->GetSystemInfo().input_connection(j, 0) << endl;
			}			
		}
		for (int k = 0; k < num_of_external_inputs; k++)
		{
			if (external_mapping(k, 0) == i)
			{
				cout << "input port # " << external_mapping(k, 1) << " is an external port mapping to # "<< k << endl;
			}		
		}
	// 
	}
	cout << "-------------------Connectivity Matrix-----------------------" << endl;
	// display the connectivity matrix
	cout << "The connectivity matrix is: " << endl;
	cout << "  to system #   ";
	for (int i = 0; i < num_of_subsystems; i++)
	{
		cout <<i <<" ";
	}
	cout << endl;
	cout << "-------------" << endl;
	for (int i = 0; i < num_of_subsystems; i++)
	{
		cout << "from system # " << i <<"|"<<connectivity.block(i, 0, 1, num_of_subsystems) << endl;
	}
}

bool SimController::PreRunProcess()
{
	bool flag = false;
	/* Logic:
	1. Parsing the external input mappiing
	a. for each subsystem, find the row index associated with -1 in the 2st colume and determine the total number of external inputs
	b. assign the external_mapping with subsystem_ID and port number
	2. Update the connectivity matirx
	*/
	// step 1 determine the external input mapping
	num_of_external_inputs = 0;
	// a. determine the total number of external inputs
	for (unsigned int i = 0; i < num_of_subsystems; i++)
	{
		for (unsigned int j = 0; j < subsystem_list[i]->GetSystemInfo().num_of_inputs; j++)
		{
			if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0) < 0)// detect if this input port is an external input
			{
				num_of_external_inputs++;
			}
		}
	}
	external_mapping.resize(num_of_external_inputs,2);
	// b. determine the external input mapping:
	int mapping_counter = 0;
	for (unsigned int i = 0; i < num_of_subsystems; i++)
	{
		for (unsigned int j = 0; j < subsystem_list[i]->GetSystemInfo().num_of_inputs; j++)
		{
			if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0) < 0)// detect if this input port is an external input
			{
				external_mapping(mapping_counter, 0) = i;
				external_mapping(mapping_counter, 1) = j;
				mapping_counter++;
			}
		}
	}
	// step 4 update the connectivity matrix
	connectivity.resize(num_of_subsystems, num_of_subsystems);
	for (unsigned int i = 0; i < num_of_subsystems; i++)// initialize
	{
		for (unsigned int j = 0; j < num_of_subsystems; j++)
		{
			connectivity(i, j) = 0;
		}
	}
	for (unsigned int i = 0; i < num_of_subsystems; i++)
	{
		for (unsigned int j = 0; j < subsystem_list[i]->GetSystemInfo().num_of_inputs; j++)
		{
			if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0) >= 0)
			{
				if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0)>= num_of_subsystems)// detect if the input mapping index exceeds the total number of subsystems
				{
					cout << "PARSING ERROR: subsystem # "<<i<<" input index "<<j<<" exceeds num_of_subsystems. "<<endl;
				}
				else
				{
					// get the num_of_outputs of the traget system
					unsigned int num_of_target_output = 0;
					num_of_target_output = subsystem_list[subsystem_list[i]->GetSystemInfo().input_connection(j, 0)]->GetSystemInfo().num_of_outputs;
					if (subsystem_list[i]->GetSystemInfo().input_connection(j, 1) >= num_of_target_output)
					{
						cout << "PARSING ERROR: subsystem # " << i << " input index " << j << " exceeds num_of_outputs of subsystem # " << subsystem_list[i]->GetSystemInfo().input_connection(j, 0) <<endl;
					}
					else
					{
						flag = true;
						connectivity(i, subsystem_list[i]->GetSystemInfo().input_connection(j, 0)) = 1;
					}
				}
			}
				
		}	
	}
	if (flag == true)
	{
		cout << "PARSING IS SUCESSFUL, READY TO RUN! " << endl;
	}
	else {
		cout << "PARSING ERROR EXISTS, CHECK SUBSYSTEM CONNECTIONS! " << endl;
	}
	return flag;
}

void SimController::GetExternalInputs(const VectorXd & extern_input)
{
	external_input_buffer = extern_input;
}

SimController::~SimController()
{
}
