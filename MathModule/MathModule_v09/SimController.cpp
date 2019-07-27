#include "stdafx.h"
#include "SimController.h"


SimController::SimController(const SolverConfig& config)
{
	solver_config = config;// load the solver information
	current_stepsize = solver_config.frame_step;
	num_of_cycles_per_step = 1;
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

int SimController::Run(const double& t, const VectorXd& extern_input)
{
	int flag = 0;
	/*Logic
	1. update the external input to the external_input butter
	2. calculate perturbed output based on butchertabealu
	3. check the residue, if the optimum step is less than the current step size, then reduce the current stepsize by
	*/
	// step 1
	GetExternalInputs(extern_input);// update external input
	// step 2
	// for each intermediate cycle, step 2 calculate the 1st update k1	
	double error_cycle = 0;
	for (int cycle = 0; cycle < num_of_cycles_per_step; cycle++)
	{
		double cycle_time = current_time + cycle * current_stepsize;// time stamp at each 
		// k1 = hf(tk, xk, uk),
		int from_system = 0;// from target  system 
		int from_index = 0;// from  target ouput of the system
		for (int i = 0; i < num_of_subsystems; i++)
		{
			for (int j = 0; j < subsystem_list[i]->GetSystemInfo().num_of_inputs; j++)
			{
				from_system = subsystem_list[i]->GetSystemInfo().input_connection(j, 0);
				if (from_system >= 0)// detect if this input port is an non-external input
				{
					// fetch input from the output of other subsystems
					from_index = subsystem_list[i]->GetSystemInfo().input_connection(j, 1);
					subsystem_list[i]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->GetOutput()(from_index));
				}
			}
			// update k1
			subsystem_list[i]->Solver_PreturbState(0, butchertableau);
			subsystem_list[i]->Solver_UpdateKiBuffer(0, cycle_time, current_stepsize, butchertableau);
		}
		// k2-kn
		for (int index_ki = 1; index_ki < solver_config.num_of_k; index_ki++)
		{
			// Preturb states
			for (int i = 0; i < num_of_subsystems; i++)
			{
				subsystem_list[i]->Solver_PreturbState(index_ki, butchertableau);
			}
			// reset these indexes:
			from_system = 0;
			from_index = 0;
			// preturb outputs of direct feed-through systems according to the update sequence
			for (int i = 0; i < num_of_subsystems; i++)
			{
				// if the system is a direct feed through system, first update inputs and then outputs. If not, directly update ouputs
				if (subsystem_list[output_sequence[i]]->GetSystemInfo().DIRECT_FEED_THROUGH)
				{
					for (int j = 0; j < subsystem_list[output_sequence[i]]->GetSystemInfo().num_of_inputs; j++)
					{
						from_system = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 0);
						if (from_system >= 0)// detect if this input port is an non-external input
						{
							// fetch input from the output of other subsystems
							from_index = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 1);
							subsystem_list[output_sequence[i]]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->Solver_GetOuputTemp()(from_index));
						}
					}
				}
				subsystem_list[output_sequence[i]]->Solver_PreturbOutput(index_ki, cycle_time, current_stepsize, butchertableau);
			}

			// update the input_temp for all non_feed_through blocks
			for (int i = 0; non_direct_feedthrough_index.size(); i++)
			{
				for (int j = 0; j < subsystem_list[non_direct_feedthrough_index[i]]->GetSystemInfo().num_of_inputs; j++)
				{
					from_system = subsystem_list[non_direct_feedthrough_index[i]]->GetSystemInfo().input_connection(j, 0);
					if (from_system >= 0)// detect if this input port is an non-external input
					{
						// fetch input from the output of other subsystems
						from_index = subsystem_list[non_direct_feedthrough_index[i]]->GetSystemInfo().input_connection(j, 1);
						subsystem_list[non_direct_feedthrough_index[i]]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->Solver_GetOuputTemp()(from_index));
					}
				}
			}

			// update the ki
			for (int i = 0; i < num_of_subsystems; i++)
			{
				subsystem_list[i]->Solver_UpdateKiBuffer(index_ki, cycle_time, current_stepsize, butchertableau);//
			}
		}
		/*------ki calculation is complete, now calculate state increment------------------------*/
		// calculate state increment and determine optimum step size
		error_cycle = 0;
		switch (solver_config.solver_type)
		{
		case DORMANDPRINCE:
			for (int i = 0; i < num_of_subsystems; i++)
			{
				if (!subsystem_list[i]->GetSystemInfo().NO_CONTINUOUS_STATE)
				{
					error_cycle += subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1, updatecoefficient2);
				}	
			}
			break;
		case RUNGKUTTA45:
			for (int i = 0; i < num_of_subsystems; i++)
			{
				subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1);
			}
			break;
		default:
			break;
		}
		// update outputs according to new state
		from_system = 0;
		from_index = 0;
		for (int i = 0; i < num_of_subsystems; i++)
		{
			subsystem_list[i]->IncrementState();// increment state first
		}
		double cycle_time_end = cycle_time + current_stepsize;
		for (int i = 0; i < num_of_subsystems; i++)
		{
			for (int j = 0; j < subsystem_list[output_sequence[i]]->GetSystemInfo().num_of_inputs; j++)
			{
				from_system = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 0);
				if (from_system >= 0)// detect if this input port is an non-external input
				{
					// fetch input from the output of other subsystems
					from_index = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 1);
					subsystem_list[output_sequence[i]]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->Solver_GetOuputTemp()(from_index));
				}
			}
			// then update output
			subsystem_list[output_sequence[i]]->UpdateOutput(cycle_time_end,)
		}
	}
	if (solver_config.adaptive_step)
	{
		double average_error = error_cycle / num_of_continuous_states;

		// determine the optimal step size
		double s = pow(solver_config.eposilon*current_stepsize/(2* average_error), 0.2);
		double optimum_step = s * current_stepsize;

		if (optimum_step< solver_config.frame_step)// if the optimum step is smaller than the frame step, equally space the frame step below the size of the optimum step;
		{
			num_of_cycles_per_step = ceil(solver_config.frame_step / optimum_step);
			current_stepsize = solver_config.frame_step / num_of_cycles_per_step;
		}
		else {
			current_stepsize = solver_config.frame_step;// if the optimum step is larger than the frame rate, then use the frame step as the step size
			num_of_cycles_per_step = 1;
		}
	}
	// update the current time
	current_time += solver_config.frame_step;
	/*------------step calculation complete------------------------*/
	return flag;
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
	a. for each subsystem, find the row index associated with -1 in the 1st colume and determine the total number of external inputs
	b. assign the external_mapping with subsystem_ID and port number
	2. Update the connectivity matirx
	3. Determine the algebraic loop (in progress)
	*/
	// step 1 determine the external input mapping
	num_of_external_inputs = 0;// total amount of external inputs
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
						//connectivity(i, subsystem_list[i]->GetSystemInfo().input_connection(j, 0)) = 1;
						connectivity(subsystem_list[i]->GetSystemInfo().input_connection(j, 0),i) = 1;
					}
				}
			}
				
		}	
	}
	DisplayTopology();
	bool isalgebraricloop = RunTopologyAnalysis();
	if (flag == true&& isalgebraricloop==false)
	{
		cout << "PARSING IS SUCESSFUL, READY TO RUN! " << endl;
		switch (solver_config.solver_type)
		{
		case DORMANDPRINCE:
			butchertableau = RungeKuttaFamily::InitbutchertableauDORMANDPRINCE();// update butcher tableau
			updatecoefficient1 = butchertableau.block(7, 1, 1, 7);
			updatecoefficient2 = butchertableau.block(8, 1, 1, 7);
			// reference: http://depa.fquim.unam.mx/amyd/archivero/DormandPrince_19856.pdf
			solver_config.num_of_k = 7;
			break;
			//case RUNGKUTTA45:
				//break;
		default:
			cout << "WARNING: INCORRECT SOLVER SETTING. THE DEFAULT SOLVER DORMANDPRINCE IS USED" << endl;
			break;
		}
		// allocate temp buffer of k_1,...,k_7
		for (int i = 0; i < num_of_subsystems; i++)
		{
			subsystem_list[i]->Solver_InitSolverBuffer(solver_config.num_of_k);
		}
	}
	else {

		if (flag == true && isalgebraricloop == true)
		{
			cout << "ALGEBRARICLOOP EXISTS, CHECK SUBSYSTEM CONNECTIONS! " << endl;
		}
		else {
			cout << "PARSING ERROR EXISTS, CHECK SUBSYSTEM CONNECTIONS! " << endl;
		}
		
	}
	return flag;
}

bool SimController::RunTopologyAnalysis()
{
	bool isAlegraricloopexist = false;
	// determine all the closed loops in the system
	TopologyAnalysis system_topology(connectivity);
	num_of_closed_loops = system_topology.RunSimulationTopologyAnalysis();// get the loop results from DFS
    output_sequence.clear();
	non_direct_feedthrough_index.clear();
	vector<bool> temp_all_susystems;
	for (int i = 0; i < num_of_subsystems; i++)
	{
		temp_all_susystems.push_back(false);
		bool inputconnected = false;
		for (int j = 0; j < num_of_subsystems;j++)// override the direct feedback info if the system is not connected to any other systems
		{
			if (connectivity(j, i) > 0)// input is connected
			{
				inputconnected = true;
				break;
			}
				
		}
		if (!inputconnected)
		{
			subsystem_list[i]->OverrideDirectFeedThroughFlag(false);// if not connected to any other subsystems, override the DIRECT_FEED_THROUGH to false
		}

	}
	for (int i = 0; i < num_of_subsystems; i++)// first segement is the non-direct feed-through block
	{
		if (!subsystem_list[i]->GetSystemInfo().DIRECT_FEED_THROUGH)
		{
			output_sequence.push_back(i);
			non_direct_feedthrough_index.push_back(i);
			temp_all_susystems[i] = true;// set true if system is a non-direct feedback system
		}
	}
	/*--------------------------------determine the algebraric loop-----------------------------------------*/
	algebraric_loops.clear();
	if (num_of_closed_loops > 0)// check the existence of algebraric loops
	{
		for (int i = 0; i < num_of_closed_loops; i++)
		{
			// determine whether the ith closed loop 
			bool isnondirectfeedthrough = false;
			for (int j = 0; j < system_topology.GetLoopIndex(i).size(); j++)
			{
				isnondirectfeedthrough = temp_all_susystems[system_topology.GetLoopIndex(i)(j)];
				if (isnondirectfeedthrough)
				{
					break;
				}
			}
			if (!isnondirectfeedthrough)
			{
				// push node index to buffer if algebraric loop exists
				isAlegraricloopexist = true;
				cout << "PARSING ERROR: ALGEBRARIC LOOP FOUND! THE LOOP CONTAINS THE FOLLOWING SUBSYSTMES: " << endl;
				for (int k = 0; k < system_topology.GetLoopIndex(i).size(); k++)
				{
					cout << " -> " << system_topology.GetLoopIndex(i)(k);
				}
				cout << endl;
				algebraric_loops.push_back(system_topology.GetLoopIndex(i));
			}

		}
	}
	/*----------------------------------------------------------------------------------------*/ 
	if (isAlegraricloopexist == false)// if no algebraric loop exists, then determine the output update sequence
	{
		if (output_sequence.size() < num_of_subsystems)// there are direct feed-through blocks
		{
			int node_left = num_of_subsystems - output_sequence.size();
			int counter = 0;
			while ((node_left>0)&&(counter< num_of_subsystems))
			{
				for (int i = 0; i< num_of_subsystems; i++)
				{
					if (temp_all_susystems[i]) continue;
					bool flag = true;
					for (int j = 0; j < num_of_subsystems; j++)
					{
						if (connectivity(j, i) > 0)
						{
							if (temp_all_susystems[j])
							{
								continue; // found a connectioin with true notation
							}
							else {
								flag = false;// found a connection with false notation
							}
						}


					}
					if (flag = false)
					{
						continue;
					}
					else {
						output_sequence.push_back(i);// now the system receiving input is a non-direct feedback system
						temp_all_susystems[i] = true;// now the system receiving input is a non-direct feedback system
						node_left--;
					}


				}
				counter++;
			}
		}
		cout << "The output sequence is : " << endl;
		for (int k = 0; k < output_sequence.size(); k++)
		{
			cout << output_sequence[k] << " -> ";
				
		}
		cout << "END" << endl;
	}
	

	return isAlegraricloopexist;
}

bool SimController::GetExternalInputs(const VectorXd & extern_input)
{
	bool flag = false;
	if (extern_input.size() == num_of_external_inputs)
	{
		for (int i = 0; i < num_of_external_inputs; i++)
		{
			subsystem_list[external_mapping(i, 0)]->Solver_UpdateInputTemp(external_mapping(i, 1), extern_input(i));// insert external inputs into buffer
		}
		flag = true;
	}
	else {
		cout << "RUN-TIME ERROR: EXTERNAL INPUTS SIZE MISMATCH" << endl;
	}
	return flag;
}

SimController::~SimController()
{
}
