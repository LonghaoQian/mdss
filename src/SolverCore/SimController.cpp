#include "pch.h"
#include "SimController.h"

namespace simulationcontrol {
	SimController::SimController(const SolverConfig& config)
	{
		solver_config = config;// load the solver information
		current_stepsize = solver_config.frame_step;
		num_of_cycles_per_step = 1; // set the number of cycles per step to 1 as initial condition
		current_time = solver_config.start_time;
		loglevel = solver_config.loglevel;

	}
	SimController::SimController() {
		// load the default parameter for solver
		solver_config.eposilon = 0.00001;
		solver_config.adaptive_step = false;
		solver_config.frame_step = 0.02;
		solver_config.mim_step = 0.005;
		solver_config.start_time = 0.0;
		solver_config.solver_type = RungeKuttaFamily::RUNGKUTTA45;
		solver_config.loggingconfig.filename = "datalog.txt";
		solver_config.loggingconfig.uselogging = false;
		solver_config.loglevel = simulationcontrol::LOGLEVEL_ERROR;
		//
		current_stepsize = solver_config.frame_step;
		num_of_cycles_per_step = 1; // set the number of cycles per step to 1 as initial condition
		current_time = solver_config.start_time;
		loglevel = solver_config.loglevel;
	}

	void SimController::EditSolverConfig(const SolverConfig & config)
	{
		solver_config = config;// load the solver information
		current_stepsize = solver_config.frame_step;
		num_of_cycles_per_step = 1; // set the number of cycles per step to 1 as initial condition
		current_time = solver_config.start_time;
		loglevel = solver_config.loglevel;
	}

	unsigned int SimController::AddSubSystem(const linearsystem::LTIParameter& parameters, const linearsystem::LTIInitialCondition& IC)
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

	unsigned int SimController::AddSubSystem(const linearsystem::IntegratorParameter & parameters, const linearsystem::IntegratorInitialCondition & IC)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::Integrator(parameters, IC));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const linearsystem::TransferFunctionParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::TransferFunction(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const linearsystem::PIDcontrollerParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new linearsystem::PIDcontroller(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const discontinuoussystem::SaturationParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new discontinuoussystem::Saturation(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const discontinuoussystem::SwitchParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new discontinuoussystem::Switch(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const dynamics::RigidBodyParameter & parameters, const dynamics::RigidBodyCondition & IC)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new dynamics::RigidBody(parameters, IC));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const mathblocks::GainParameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Gain(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const mathblocks::ConstantParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Constant(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const mathblocks::SumParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Sum(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const mathblocks::MultiplicationParam & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Multiplication(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubsystem(const mathblocks::TrigonometryParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::TrigonometricFunction(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubsystem(const mathblocks::Lookup1DParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Lookup1D(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubsystem(const mathblocks::Lookup2DParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new mathblocks::Lookup2D(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const  source_sink::PeriodicWaveparameter& parameters) {
		subsystem_info system_info;
		subsystem_list.emplace_back(new  source_sink::PeriodicWave(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const geographic::StandardAtmosphereParameter& parameters) {
		subsystem_info system_info;
		subsystem_list.emplace_back(new geographic::StandardAtmosphere(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const geographic::GravityModelParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new geographic::Gravity(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const aero::AerosForceParameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new aero::AeroForceMoment1(parameters));
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const aero::AeroAngleParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new aero::AeroAngle(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const source_sink::Stepparameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new source_sink::Step(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const source_sink::Rampparamter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new source_sink::Ramp(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const  propulsionsystem::CFM56Parameter& parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new  propulsionsystem::CFM56AuxiliaryModel(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	unsigned int SimController::AddSubSystem(const propulsionsystem::CF56ThrustModelParameter & parameters)
	{
		subsystem_info system_info;
		subsystem_list.emplace_back(new  propulsionsystem::CFM56ThrustModel(parameters));
		system_info = subsystem_list.back()->GetSystemInfo();
		// update number of subsystems
		num_of_subsystems = subsystem_list.size();
		// returen subystem handle
		return CreateSystemHandle(system_info, subsystem_list);
	}

	void SimController::EditConnectionMatrix(unsigned int handleID,
											 unsigned int from_input_ID,
											 unsigned int to_output_systemID, 
											 unsigned int to_output_portID)
	{
		if (handleID >= num_of_subsystems) { // if the handle ID is greater than the total number of subsystems, return an error msg
			if (solver_config.loglevel >= LOGLEVEL_ERROR) {
				std::cout << " CONNNECTION ERROR: THE HANDLE ID IS: " << handleID << ". HOWEVER, THE TOTAL NUMBER OF SUBSYSTEM IS: " << num_of_subsystems << "\n";
			}
		}
		else {
			if (from_input_ID >= subsystem_list[handleID]->GetSystemInfo().num_of_inputs) {
				if (solver_config.loglevel >= LOGLEVEL_ERROR) {
					std::cout << " CONNNECTION ERROR: THE INPUT PORT ID IS: " 
							  << from_input_ID << ". HOWEVER, THE TOTAL NUMBER OF INPUT FOR THE SYSTEM IS: "
							  << subsystem_list[handleID]->GetSystemInfo().num_of_inputs << "\n";
				}
			}
			else {
				system_handle_list[(std::size_t)handleID]->input_connection_list(from_input_ID, simulationcontrol::subsystemID)  = to_output_systemID;
				system_handle_list[(std::size_t)handleID]->input_connection_list(from_input_ID, simulationcontrol::outputportID) = to_output_portID;
			}
		}

	}

	bool SimController::MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping)
	{
		// parse the connection mapping 
		// the connection mapping is defined in a Xx2 matrix
		// index of the input port, system ID of the output, index of the output 
		// TO DO: add unconnected to port
		bool flag = false;
		if (system_ID >= num_of_subsystems)
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

	bool SimController::MakeConnection(const subsystem_handle & handle)
	{
		// parse the connection mapping based on system handle
		// the connection mapping is defined in a Xx2 matrix
		// index of the input port, system ID of the output, index of the output 
		bool flag = false;
		if (handle.ID >= num_of_subsystems)
		{
			cout << "CONNECTION ERROR: The system ID goes out of bound, the number of subsystem is " << num_of_subsystems << endl;
			cout << " while the input system ID is (system_ID<number of subsystem)" << handle.ID << endl;
		}
		else {
			if (handle.input_connection_list.rows() == subsystem_list[handle.ID]->GetSystemInfo().num_of_inputs) {
				subsystem_list[handle.ID]->SetInputConnection(handle.input_connection_list);
				flag = true;
			}
			else {
				cout << "CONNECTION ERROR: The connection mapping matrix does not match the number of inputs of system # " << handle.ID << endl;
			}

		}
		return flag;
	}

	bool SimController::FlushMakeConnection()
	{
		bool flag = true;
		connection_flag_list.resize(num_of_subsystems, 1);
		for (int i = 0; i < num_of_subsystems; i++) {
			bool flag_for_each = MakeConnection(*system_handle_list[i]);
			connection_flag_list(i) = flag_for_each;
			if (!flag_for_each) {
				flag = false;
			}
		}
		
		if (solver_config.loglevel >= LOGLEVEL_ERROR) {
			if (!flag) {
				std::cout << " CONNNECTION ERROR! CHECK ERROR MESSAGE  \n";
			}
		}

		return flag;
	}

	subsystem_handle SimController::GetSystemHandle(const unsigned int system_ID)
	{
		if (system_ID < num_of_subsystems) {
			return *system_handle_list[system_ID];
		}
		else {
			subsystem_handle empty_handel;
			empty_handel.ID = -1;
			if (solver_config.loglevel >= LOGLEVEL_WARN) {
				std::cout << " WARN: SYSTEM ID GREATER THAN TOTAL NUMBER OF SYSTEMS  \n";
			}
			return empty_handel;
		}
		
	}

	int SimController::Run_Update(const VectorXd& extern_input)
	{
		/*------------ log data ---------------------------------------*/		
		if (solver_config.loggingconfig.uselogging) {
			LogRequestedData();
		}
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

	bool SimController::DefineDataLogging(const unsigned int output_system_ID, const unsigned int output_port_ID, string tag)
	{
		bool flag = false;
		if (output_system_ID > num_of_subsystems)
		{
			cout << " Logger defnition error: The total number of subsystem is: " << num_of_subsystems << endl;
			cout << " while the output system ID is (system_ID<number of subsystem)" << output_system_ID << endl;
		}
		else {
			if (output_port_ID < subsystem_list[output_system_ID]->GetSystemInfo().num_of_outputs) {
				Matrix<int, 1, 2> temp_port;
				temp_port(subsystemID)  = output_system_ID;
				temp_port(outputportID) = output_port_ID;
				logportlist.push_back(temp_port);
				logtaglist.push_back(tag);
				flag = true;
			}
			else {
				cout << "Logger defnition error: The requested port ID #"<< output_port_ID <<" is larger than the number of outputs of the system # " << output_system_ID << endl;
			}

		}
		return flag;
	}

	LoggerTag SimController::GetLoggerTag(unsigned int TagIndex)
	{
		if (!solver_config.loggingconfig.uselogging) {
			return LoggerTag{"LOGGER IS OFF!",0,0};
		}
		else {
			int num_of_tags = logtaglist.size();
			if (TagIndex >= num_of_tags) {
				return LoggerTag{ "INDEX OUT OF RANGE! ",0,0 };
			}
			else
			{
				LoggerTag logger_tag_;
				logger_tag_.output_port_ID = logportlist[TagIndex](outputportID);
				logger_tag_.output_system_ID = logportlist[TagIndex](subsystemID);
				logger_tag_.tag = logtaglist[TagIndex];
				return logger_tag_;
			}
		}
	}

	void SimController::DisplayLoggerTagList()
	{
		if (solver_config.loggingconfig.uselogging) {
			if (logtaglist.size() < 1) {
				cout << " LOGGER IS TURNED ON, BUT NOT TAGS ARE DEFINED! \n";
			}
			else {
				LoggerTag logger_tag_;
				cout << " LOGGER IS TURNED ON, THE FOLLOWING TOPICS ARE RECORDED: \n";
				for (unsigned int i = 0; i < logtaglist.size();i++) {
					logger_tag_ = GetLoggerTag(i);
					cout << logger_tag_.tag << " RECORDS " << logger_tag_.output_port_ID << " TH OUTPUT OF SYSTEM # " << logger_tag_.output_system_ID << "\n";
				}
			}
		}
		else
		{
			cout << " LOGGER IS TURNED OFF! \n";
		}
	}

	int SimController::PostRunProcess()
	{
		std::cout << "CALCULATION COMPLETE! " << std::endl;
		// display some logging info after simulation
		if (solver_config.loggingconfig.uselogging) {
			if (loggingdata.is_open()) {
				LogRequestedData();// log the final frame of simulation
				std::cout << "SELECTED DATA HAS BEEN LOGGED TO: "<< solver_config.loggingconfig.filename << std::endl;
				loggingdata.close();
			}
		}// close file
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
						if (solver_config.loglevel >= LOGLEVEL_ERROR) {
							cout << "PARSING ERROR: subsystem # " << i << " input index " << j << " exceeds num_of_subsystems. " << endl;
						}
						flag = false;
					}
					else
					{
						// get the num_of_outputs of the traget system
						unsigned int num_of_target_output = 0;
						num_of_target_output = subsystem_list[subsystem_list[i]->GetSystemInfo().input_connection(j, 0)]->GetSystemInfo().num_of_outputs;
						if (subsystem_list[i]->GetSystemInfo().input_connection(j, 1) >= num_of_target_output)
						{
							if (solver_config.loglevel >= LOGLEVEL_ERROR) {
								cout << "PARSING ERROR: subsystem # " << i << " input index " << j << " exceeds num_of_outputs of subsystem # " << subsystem_list[i]->GetSystemInfo().input_connection(j, 0) << endl;
							}
							flag = false;
						}
						else
						{
							connectivity(subsystem_list[i]->GetSystemInfo().input_connection(j, 0), i) = 1;
						}
					}
				}

			}
		}
		if (solver_config.loglevel >= LOGLEVEL_ALL) {
			DisplayTopology(); // display information if log level is set to ALL
		}
		bool isalgebraricloop = RunTopologyAnalysis(); // Do the algebraric loop check before loading the butcher tabeleau
		// if no algebraric loop discovered, load the table according to the solver choice
		if (flag == true && isalgebraricloop == false)
		{
			cout << "SOLVER PARSING IS SUCESSFUL! LOADING BUTCHERTABLEAU... " << endl;
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
			// write the header information to logged file
			// push the initial condition into the logger
			if (solver_config.loggingconfig.uselogging) {
				total_number_log = logportlist.size();
				// if the logging setting is true, turn on open the log file
				loggingdata.open(solver_config.loggingconfig.filename, std::ios::trunc);// overwrite exsiting file
				// write the file info
				if (loggingdata.is_open()) {
					// insert identifier as the first row:
					loggingdata << "This file is a data log from solver." << "\n";
					// insert number of data tags as the second row:
					loggingdata << total_number_log << "\n";
					// the insert each tag on an individual row:
					loggingdata << "t" << "\n";
					for (int i = 0; i < total_number_log; i++) {
						loggingdata << logtaglist[i] << "\n";
					}
				}
			}
		}
		else {

			if (flag == true && isalgebraricloop == true)
			{
				if (solver_config.loglevel >= LOGLEVEL_ERROR) {
					cout << "ALGEBRARICLOOP EXISTS, CHECK SUBSYSTEM CONNECTIONS! " << endl;
				}
			}
			else {
				if (solver_config.loglevel >= LOGLEVEL_ERROR) {
					cout << "PARSING ERROR EXISTS, CHECK SUBSYSTEM CONNECTIONS! " << endl;
				}
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
		bool display_closed_loops = false;
		if (solver_config.loglevel >= LOGLEVEL_ALL) {
			display_closed_loops = true;
		}
		num_of_closed_loops = system_topology.RunSimulationTopologyAnalysis(display_closed_loops);// get the loop results from DFS
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
					if (solver_config.loglevel>=LOGLEVEL_ERROR) {
							cout << "PARSING ERROR: ALGEBRARIC LOOP FOUND! THE LOOP CONTAINS THE FOLLOWING SUBSYSTMES: " << endl;
						// display algebraic loops
						for (int k = 0; k < system_topology.GetLoopIndex(i).size(); k++)
						{
							cout << " -> " << system_topology.GetLoopIndex(i)(k);
						}
						cout << endl;
					}
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
			if (solver_config.loglevel >= LOGLEVEL_ALL) {
				// after the output sequence is determined, show it. 
				cout << "The output sequence is : " << endl;
				for (int k = 0; k < output_sequence.size(); k++) {
					cout << output_sequence[k] << " -> "; 
				}
				cout << "END" << endl;
			}
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

	unsigned int SimController::CreateSystemHandle(const subsystem_info & info, const vector<unique_ptr<Subsystem>>& subsystem_list)
	{
		system_handle_list.emplace_back(new subsystem_handle);// add a handle on the heap
		system_handle_list.back()->isParameterOK = info.system_parameter_ok;
		system_handle_list.back()->label_ = info.label_;
		system_handle_list.back()->type = info.type;
		system_handle_list.back()->ID = subsystem_list.size() - 1;
		if (subsystem_list.back()->GetSystemInfo().num_of_inputs > 0)
		{
			system_handle_list.back()->input_connection_list.resize(subsystem_list.back()->GetSystemInfo().num_of_inputs,2); // resize and initialize the coonection matrix
			for (int i = 0; i < subsystem_list.back()->GetSystemInfo().num_of_inputs; i++) {
				system_handle_list.back()->input_connection_list(i, simulationcontrol::subsystemID) = simulationcontrol::external;
				system_handle_list.back()->input_connection_list(i, simulationcontrol::outputportID) = 0;
			}
		}
		return (unsigned int)system_handle_list.size() - 1;
	}

	void SimController::LogRequestedData()
	{
		if (loggingdata.is_open()) {
			loggingdata << Run_GetSystemTime();
			for (int i = 0; i < total_number_log; i++) {
				loggingdata << "*" << Run_GetSubsystemOuput(logportlist[i](subsystemID))(logportlist[i](outputportID));
			}
			loggingdata << "\n"; //"\n" is madatory for fgetl to execute properly in matlab
		}
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
