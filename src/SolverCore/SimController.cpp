#include "pch.h"
#include "SimController.h"

namespace simulationcontrol {
	SimController::SimController(const SolverConfig& config)
	{
		solver_config = config;// load the solver information
		current_stepsize = solver_config.frame_step;
		num_of_cycles_per_step = 1; // set the number of cycles per step to 1 as initial condition
		current_time = solver_config.start_time;
	}



	subsystem_handle SimController::AddSubSystem(const linearsystem::LTIParameter& parameters, const linearsystem::LTIInitialCondition& IC)
	{
		/* Logic:
		1. push a pointer into subsystem_list
		2. load parameters
		3. load initial conditions
		4. check the connectivity
		5. update the connnectivity matrix
		*/
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::LTIsystem(parameters, IC));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const linearsystem::IntegratorParameter & parameters, const linearsystem::IntegratorInitialCondition & IC)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::Integrator(parameters, IC));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const linearsystem::TransferFunctionParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::TransferFunction(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const RigidBodyParameter & parameters, const RigidBodyCondition & IC)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new RigidBody(parameters, IC));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const mathblocks::GainParameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Gain(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const mathblocks::ConstantParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Constant(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const mathblocks::SumParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Sum(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const mathblocks::MultiplicationParam & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Multiplication(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const  source_sink::PeriodicWaveparameter& parameters) {
		subsystem_info system_info;
		subsystem_list.emplace_back(new  source_sink::PeriodicWave(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const geographic::StandardAtmosphereParameter& parameters) {
		subsystem_info system_info;
		subsystem_list.emplace_back(new geographic::StandardAtmosphere(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const geographic::GravityModelParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new geographic::Gravity(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const aero::AerosForceParameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new aero::AeroForceMoment1(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const aero::AeroAngleParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new aero::AeroAngle(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const source_sink::Stepparameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new source_sink::Step(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	subsystem_handle SimController::AddSubSystem(const source_sink::Rampparamter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new source_sink::Ramp(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	bool SimController::MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping)
	{
		// parse the connection mapping 
		// the connection mapping is defined in a Xx2 matrix
		// index of the input port, system ID of the output, index of the output 
		// TO DO: add unconnected to port
		bool flag = false;
		if (system_ID > num_of_subsystems)
		{
			cout << "CONNECTION ERROR: The system ID goes out of bound, the number of subsystem is " << num_of_subsystems << endl;
			cout << " while the input system ID is (system_ID<number of subsystem)" << system_ID << endl;
		}
		else {
			if (connection_mapping.rows() == subsystem_list[system_ID]->GetSystemInfo().num_of_inputs) {
				subsystem_list[system_ID]->SetInputConnection(connection_mapping);
				flag = true;
			}
			else {
				cout << "CONNECTION ERROR: The connection mapping matrix does not match the number of inputs of system # " << system_ID << endl;
			}

		}

		return flag;
	}

	int SimController::Run_Update(const VectorXd& extern_input)
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
		// for each intermediate cycle, calculate increments using numerical method
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
				if (!subsystem_list[i]->GetSystemInfo().NO_CONTINUOUS_STATE)
				{
					subsystem_list[i]->Solver_UpdateKiBuffer(0, cycle_time, current_stepsize, butchertableau);
				}
			}
			// k2-kn
			for (int index_ki = 1; index_ki < solver_config.num_of_k; index_ki++)
			{
				// Preturb states
				for (int i = 0; i < num_of_subsystems; i++)
				{
					if (!subsystem_list[i]->GetSystemInfo().NO_CONTINUOUS_STATE)
					{
						subsystem_list[i]->Solver_PreturbState(index_ki, butchertableau);
					}
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
				for (int i = 0; i < non_direct_feedthrough_index.size(); i++)
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
					if (!subsystem_list[i]->GetSystemInfo().NO_CONTINUOUS_STATE)
					{
						subsystem_list[i]->Solver_UpdateKiBuffer(index_ki, cycle_time, current_stepsize, butchertableau);//
					}
				}
			}
			/*------ki calculation is complete, now calculate state increment------------------------*/
			// calculate state increment and determine optimum step size
			error_cycle = 0;
			switch (solver_config.solver_type)
			{
			case RungeKuttaFamily::DORMANDPRINCE: // for dormand prince, it has 2 update formulars 
				for (int i = 0; i < num_of_subsystems; i++)
				{
					if (!subsystem_list[i]->GetSystemInfo().NO_CONTINUOUS_STATE)
					{
						error_cycle += subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1, updatecoefficient2);
					}
				}
				break;
			case RungeKuttaFamily::RUNGKUTTA45: // for other rungkutta method, there is only one update formular
				for (int i = 0; i < num_of_subsystems; i++)
				{
					subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1);
				}
				break;
			case RungeKuttaFamily::EULER1ST:
				for (int i = 0; i < num_of_subsystems; i++)
				{
					subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1);
				}
				break;
			default:
				for (int i = 0; i < num_of_subsystems; i++)
				{
					subsystem_list[i]->Solver_CalculateIncrement(updatecoefficient1);
				}
				break;
			}
			// update outputs according to new state
			from_system = 0;
			from_index = 0;
			for (int i = 0; i < num_of_subsystems; i++)
			{
				subsystem_list[i]->IncrementState();// increment state first
			}
			for (int i = 0; i < num_of_subsystems; i++)
			{
				for (int j = 0; j < subsystem_list[output_sequence[i]]->GetSystemInfo().num_of_inputs; j++)
				{
					from_system = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 0);
					if (from_system >= 0)// detect if this input port is an non-external input
					{
						// fetch input from the output of other subsystems
						from_index = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 1);
						subsystem_list[output_sequence[i]]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->GetOutput()(from_index));
					}
				}
				// then update output
				subsystem_list[output_sequence[i]]->UpdateOutput(cycle_time, current_stepsize);
			}
		}
		if (solver_config.adaptive_step && solver_config.solver_type == RungeKuttaFamily::DORMANDPRINCE)
		{
			double average_error = error_cycle / num_of_continuous_states;

			// determine the optimal step size
			double s = pow(solver_config.eposilon / (2 * average_error), 0.2);
			double optimum_step = s * current_stepsize;
			/*if the optimum step is smaller than the frame step,
			  then equally separate the framestep to multiple ministeps
			*/
			if (optimum_step < solver_config.frame_step)
			{
				/*If the optimum step is smaller than the minmum step, then use minimum step*/
				if (optimum_step < solver_config.mim_step)
				{
					optimum_step = solver_config.mim_step;
				}

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

	double SimController::Run_GetSystemTime()
	{
		return current_time;
	}

	VectorXd SimController::Run_GetSubsystemOuput(const unsigned int system_ID)
	{
		return subsystem_list[system_ID]->GetOutput();
	}

	int SimController::PostRunProcess()
	{
		return 0;
	}

	void SimController::DisplayTopology()
	{
		cout << "The total Number of Subsystems is " << num_of_subsystems << endl;
		subsystem_info system_info;
		for (int i = 0; i < num_of_subsystems; i++)
		{
			cout << "+++++++++++++++++++++++++++++++++++++++++++" << endl;
			system_info = subsystem_list[i]->GetSystemInfo();

			cout << " Subsystem # " << i << " is of type " << GetSystemTypeFromID(system_info.type) << endl;
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
					cout << "input port # " << external_mapping(k, 1) << " is an external port mapping to # " << k << endl;
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
			cout << i << " ";
		}
		cout << endl;
		cout << "-------------" << endl;
		for (int i = 0; i < num_of_subsystems; i++)
		{
			cout << "from system # " << i << "|" << connectivity.block(i, 0, 1, num_of_subsystems) << endl;
		}
	}

	void SimController::ReshapeExternalInputVector(VectorXd& extern_input)
	{
		extern_input.resize(num_of_external_inputs);
		extern_input.setZero();
	}

	bool SimController::PreRunProcess()
	{
		bool flag = true;
		// TO DO: add a function to distribute blocks in subsystem groups
		/* Logic:
		1. Parsing the external input mappiing
		a. for each subsystem, find the row index associated with -1 in the 1st colume and determine the total number of external inputs
		b. assign the external_mapping with subsystem_ID and port number
		2. Update the connectivity matirx
		3. Determine the algebraic loop
		*/
		// step 1 determine the external input mapping
		num_of_external_inputs = 0;// total amount of external inputs
		num_of_continuous_states = 0;// total number of continuous states
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
		external_mapping.resize(num_of_external_inputs, 2);
		for (unsigned int i = 0; i < num_of_subsystems; i++)
		{
			num_of_continuous_states += subsystem_list[i]->GetSystemInfo().num_of_continuous_states;
		}

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
					if (subsystem_list[i]->GetSystemInfo().input_connection(j, 0) >= num_of_subsystems)// detect if the input mapping index exceeds the total number of subsystems
					{
						cout << "PARSING ERROR: subsystem # " << i << " input index " << j << " exceeds num_of_subsystems. " << endl;
						flag = false;
					}
					else
					{
						// get the num_of_outputs of the traget system
						unsigned int num_of_target_output = 0;
						num_of_target_output = subsystem_list[subsystem_list[i]->GetSystemInfo().input_connection(j, 0)]->GetSystemInfo().num_of_outputs;
						if (subsystem_list[i]->GetSystemInfo().input_connection(j, 1) >= num_of_target_output)
						{
							cout << "PARSING ERROR: subsystem # " << i << " input index " << j << " exceeds num_of_outputs of subsystem # " << subsystem_list[i]->GetSystemInfo().input_connection(j, 0) << endl;
							flag = false;
						}
						else
						{
							//connectivity(i, subsystem_list[i]->GetSystemInfo().input_connection(j, 0)) = 1;
							connectivity(subsystem_list[i]->GetSystemInfo().input_connection(j, 0), i) = 1;
						}
					}
				}

			}
		}
		DisplayTopology();
		bool isalgebraricloop = RunTopologyAnalysis(); // Do the algebraric loop check before loading the butcher tabeleau
		// if no algebraric loop discovered, load the table according to the solver choice
		if (flag == true && isalgebraricloop == false)
		{
			cout << "PARSING IS SUCESSFUL! LOADING BUTCHERTABLEAU... " << endl;
			RungeKuttaFamily::LoadButcherTableau(solver_config.solver_type,
				butchertableau,
				updatecoefficient1,
				updatecoefficient2,
				solver_config.num_of_k);
			// allocate temp buffer of k_1,...,k_j
			for (int i = 0; i < num_of_subsystems; i++)
			{
				subsystem_list[i]->Solver_InitSolverBuffer(solver_config.num_of_k);
			}
			// calculate the initial outputs based on initial conditions
			int from_system = 0;
			int from_index = 0;
			for (int i = 0; i < num_of_subsystems; i++)
			{
				for (int j = 0; j < subsystem_list[output_sequence[i]]->GetSystemInfo().num_of_inputs; j++)
				{
					from_system = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 0);
					if (from_system >= 0)// detect if this input port is an non-external input
					{
						// fetch input from the output of other subsystems
						from_index = subsystem_list[output_sequence[i]]->GetSystemInfo().input_connection(j, 1);
						subsystem_list[output_sequence[i]]->Solver_UpdateInputTemp(j, subsystem_list[from_system]->GetOutput()(from_index));
					}
				}
				// then update output
				subsystem_list[output_sequence[i]]->UpdateOutput(solver_config.start_time, 0);
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
	bool SimController::DetermineOutputSequenceDFS(int level, unsigned int index_now) {
		if (level > number_of_not_ready) // if the layer goes beyond number of total 
		{
			cout << "PARSING ERROR EXISTS, UPDATE OUTPUT SEQUENCE FAILED!" << endl;
				return false;
		}
		bool only_connected_to_ready_system = true;
		int input_not_ready_system = 0;
		for (int j = 0; j < num_of_subsystems; j++) {
			if (connectivity(j, index_now) > 0) {
				if (!temp_all_susystems[j]) {
					only_connected_to_ready_system = false;// found a connection with false notation
					input_not_ready_system = j;// and record the index of the not ready subsystem from which the input port is connected
					break;
				}
			}
		}
		if (only_connected_to_ready_system) {
			// if only connected to ready_system, them update the output sequence, and set the index on the templist to true
			output_sequence.push_back(index_now);
			temp_all_susystems[index_now] = true;
		}
		else {
			// if there is connection to not_ready_system, 
			DetermineOutputSequenceDFS(level + 1, input_not_ready_system);// then the input_not_ready_system is used as the index_now for the next layer
		}
		return true;
	}
	/*
	The topology analysis function:
	1. determine whether an algbraic loop exits in the system 
	2. if no algbraic loop exits, determine the block output update sequence based on connection
	*/
	bool SimController::RunTopologyAnalysis()
	{
		bool isAlegraricloopexist = false;
		// determine all the closed loops in the system
		TopologyAnalysis system_topology(connectivity);
		num_of_closed_loops = system_topology.RunSimulationTopologyAnalysis();// get the loop results from DFS
		output_sequence.clear();				// the order on which the solver uploads the output of each subsystem
		non_direct_feedthrough_index.clear();	// this stores all the indexes for non direct feedthrough blocks
		temp_all_susystems.clear();		// this stores wether a subsystem in the output sequence is ready
		number_of_not_ready = 0;
		/*
		step 1. if a direct feed-through block only gets external input, then override it to a non-direct feedthrough block
		*/
		for (int i = 0; i < num_of_subsystems; i++) {
			temp_all_susystems.push_back(false);
			bool inputconnected = false;
			/*
			scan through the connectivity matrix to see if the block is unconnected
			*/
			for (int j = 0; j < num_of_subsystems; j++) {
				if (connectivity(j, i) > 0)// if input is connected, then break the loop and 
				{
					inputconnected = true; 
					break;
				}
			}
			if (!inputconnected) // if the system is unconnected, then all its inputs are external and is overritten to a non direct feed-through block
			{
				subsystem_list[i]->OverrideDirectFeedThroughFlag(false);// if not connected to any other subsystems, override the DIRECT_FEED_THROUGH to false
			}
		}
		/*
		step 2. set the first segement of output seqence to non non-direct feed-through block
		*/
		for (int i = 0; i < num_of_subsystems; i++)// first segement is the non-direct feed-through block
		{
			if (!subsystem_list[i]->GetSystemInfo().DIRECT_FEED_THROUGH)
			{
				output_sequence.push_back(i);
				non_direct_feedthrough_index.push_back(i);
				temp_all_susystems[i] = true;// set true if system is a non-direct feedback system (ready for outout sequence)
			}
			else {
				number_of_not_ready++; // if the system is not ready, increment the counter by 1
			}
		}
		/* step 3. determine whether an algebraric loop exits*/
		algebraric_loops.clear();
		if (num_of_closed_loops > 0)// check the existence of algebraric loops
		{
			for (int i = 0; i < num_of_closed_loops; i++)
			{
				// determine whether the ith closed loop  only contains direct feed-through blocks
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
		/* step 4. if no algbraric loop exists, then determine the output update order*/
		if (isAlegraricloopexist == false)// if no algebraric loop exists, then determine the output update sequence
		{
			if (output_sequence.size() < num_of_subsystems)// there are direct feed-through blocks
			{
				int counter = 0;
				bool not_ready_subsystem_exists = true;
				while (not_ready_subsystem_exists && (counter < num_of_subsystems))
				{
					auto it = find(temp_all_susystems.begin(), temp_all_susystems.end(), false); // scan the temp list for not-ready subsystem
					if (it != temp_all_susystems.end()) {
						// if there is not ready subsystem, them determine whether is it only connected to ready subsystem
						int index_base = distance(temp_all_susystems.begin(), it); // then the index is used as the base for the DFS
						DetermineOutputSequenceDFS(0, index_base);
					}
					else {
						not_ready_subsystem_exists = false; // if all the subsystems are ready exit the while loop
					}
					counter++;
				}
			}
			// after the output sequence is determined, show it. 
			cout << "The output sequence is : " << endl;
			for (int k = 0; k < output_sequence.size(); k++) {
				cout << output_sequence[k] << " -> "; // TO DO: display the output sequence
			}
			cout << "END" << endl;
		}
		return isAlegraricloopexist;
	}

	string SimController::GetSystemTypeFromID(subsystem_type type)
	{
		auto name = subsystem_type_list.find(type);
		if (name != subsystem_type_list.end()) {
			return name->second;
		}
		else {
			return "Undefined Type. Did you forget to add the name to subsystem_type_list? ";
		}
	}

	subsystem_handle SimController::CreateSystemHandle(const subsystem_info & info, const vector<unique_ptr<Subsystem>>& subsystem_list)
	{
		subsystem_handle handle;
		handle.isParameterOK = info.system_parameter_ok;
		handle.label_ = info.label_;
		handle.type = info.type;
		handle.ID = subsystem_list.size() - 1;
		return  handle;
	}

	bool SimController::GetExternalInputs(const VectorXd & extern_input)
	{
		bool flag = false;
		if (num_of_external_inputs > 0)
		{
			if (extern_input.size() == num_of_external_inputs)
			{
				// insert external inputs into buffer
				for (int i = 0; i < num_of_external_inputs; i++)
				{
					subsystem_list[external_mapping(i, 0)]->Solver_UpdateInputTemp(external_mapping(i, 1), extern_input(i));
				}
				flag = true;
			}
			else {
				cout << "RUN-TIME ERROR: EXTERNAL INPUTS SIZE MISMATCH" << endl;
			}
		}
		else {
			flag = true;// if there is no external inputs, then do nothing
		}

		return flag;
	}

	SimController::~SimController()
	{
	}
}
