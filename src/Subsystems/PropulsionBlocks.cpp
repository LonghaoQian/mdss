#include "pch.h"
#include "PropulsionBlocks.h"
namespace propulsionsystem {
	CFM56AuxiliaryModel::CFM56AuxiliaryModel(const CFM56Parameter& param) {
		parameter = param;
		invN1Tf = 1.0 / parameter.CFM56N1model.Tf;
		invEGTTf = 1.0 / parameter.CFM56EGTmodel.Tf;
		invFFTf = 1.0 / parameter.CFM56FuelFlowmodel.Tf;
		// sets the system type and category 
		system_info.category = PROPULSION;
		system_info.type = propulsion_CFM56AUXILIARYMODEL;
		// set ready_to_run to true since there is not parameter check for this subsystem
		ready_to_run = true;
		// determine the size of the system
		system_info.num_of_continuous_states = 3; // N1 EGT FF
		system_info.num_of_inputs = 1; // N2
		system_info.num_of_outputs = 3; // N1 EGT FF
		// check the compatibility of the system matrices:
		system_info.system_parameter_ok = true;
		system_info.DIRECT_FEED_THROUGH = false;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = false;
		// initialize state memeory
		state.resize(system_info.num_of_continuous_states);
		state.setZero();
		state(CFM56_OUTPUT_EGT) = parameter.CFM56EGTmodel.environment_temp;
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
	}
	CFM56AuxiliaryModel::~CFM56AuxiliaryModel() {

	}
	void CFM56AuxiliaryModel::DifferentialEquation(const double& t,
												   const VectorXd& state,
												   const VectorXd& input,
												   VectorXd& derivative) {
		// N1 model
		N2_d = input(0) - parameter.CFM56N1model.b0;
		if (N2_d <= 0.0) {
			N2_d = 0.0;
		}
		if (input(0) > parameter.CFM56N1model.b1) {
			if (input(0) > parameter.CFM56N1model.b2) {
				N1_input = parameter.CFM56N1model.k2*(input(0) - parameter.CFM56N1model.b2) + parameter.CFM56N1model.c2;
			}
			else {
				N1_input = parameter.CFM56N1model.k1*(input(0) - parameter.CFM56N1model.b1) + parameter.CFM56N1model.c1;
			}
		}
		else {
			N1_input = parameter.CFM56N1model.k0*pow(N2_d, parameter.CFM56N1model.power0);
		}
		derivative(CFM56_OUTPUT_N1) = invN1Tf * (N1_input -state(CFM56_OUTPUT_N1));
		// EGT model
		if (input(0)> parameter.CFM56EGTmodel.b1) {
			N2_c = input(0) - parameter.CFM56EGTmodel.b1;
			EGT_input = parameter.CFM56EGTmodel.k1*N2_c + parameter.CFM56EGTmodel.c1;
		}
		else {
			N2_c = input(0) - parameter.CFM56EGTmodel.b0;
			if (N2_c <= 0.0)
			{
				N2_c = 0.0;
			}
			EGT_input = N2_c * parameter.CFM56EGTmodel.k0;
			if (EGT_input <  parameter.CFM56EGTmodel.environment_temp) {
				EGT_input = parameter.CFM56EGTmodel.environment_temp;
			}
			
		}

		derivative(CFM56_OUTPUT_EGT) = invEGTTf* (EGT_input - state(CFM56_OUTPUT_EGT));
		// FF model
		if (input(0) > parameter.CFM56FuelFlowmodel.b0) {
			if (input(0) > parameter.CFM56FuelFlowmodel.b1) {
				FF_input = parameter.CFM56FuelFlowmodel.c1 +
					parameter.CFM56FuelFlowmodel.k1 * pow(parameter.CFM56FuelFlowmodel.d1*(input(0) - parameter.CFM56FuelFlowmodel.b1), parameter.CFM56FuelFlowmodel.power1);
			}
			else {
				FF_input = parameter.CFM56FuelFlowmodel.c0 + parameter.CFM56FuelFlowmodel.k0 *(input(0) - parameter.CFM56FuelFlowmodel.b0);
			}
		}
		else {
			FF_input = 0.0;
		}

		derivative(CFM56_OUTPUT_FF) = invFFTf * (FF_input - state(CFM56_OUTPUT_FF));
		
	}
	void CFM56AuxiliaryModel::OutputEquation(const double& t,
											 const VectorXd& state,
											 const VectorXd& input, VectorXd& output) {
		output = state;

	}
	void CFM56AuxiliaryModel::IncrementState() {
		state += solver_buffer_state_increment1;
	}

	void CFM56AuxiliaryModel::DisplayParameters() {
		std::cout << "-------------" << std::endl;

		std::cout << "N1 Tf is : " << parameter.CFM56N1model.Tf << "\n";
		std::cout << "N1 b0 is : " << parameter.CFM56N1model.b0 << "\n";
		std::cout << "N1 b1 is : " << parameter.CFM56N1model.b1 << "\n";
		std::cout << "N1 k0 is : " << parameter.CFM56N1model.k0 << "\n";
		std::cout << "N1 power0 is : " << parameter.CFM56N1model.power0 << "\n";
		std::cout << "N1 k1 is : " << parameter.CFM56N1model.k1 << "\n";
		std::cout << "N1 c1 is : " << parameter.CFM56N1model.c1 << "\n";
		std::cout << "N1 k2 is : " << parameter.CFM56N1model.k2 << "\n";
		std::cout << "N1 c2 is : " << parameter.CFM56N1model.c2 << "\n";
	}
	void CFM56AuxiliaryModel::DisplayInitialCondition() {
		std::cout << "----Initial condition is set to zero form CFM56 Auxiliary model-----" << std::endl;
	}

	CFM56ThrustModel::CFM56ThrustModel(const CF56ThrustModelParameter& param) {
		parameter = param;
		N1_dff = parameter.MaxN1 - parameter.IdelN1;
		// sets the system type and category 
		system_info.category = PROPULSION;
		system_info.type = propulsion_CFM56THRUST;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = true;
		system_info.num_of_continuous_states = 0; //
		system_info.num_of_inputs = 3; // N1 Mach Height
		system_info.num_of_outputs = 1; // Thrust in N
		// check the consistency of the thrust table:

		// col = Height.
		// row = Mach;

		auto Mach_index = parameter.Mach.size(); // number of data points for row reference
		auto Height_index = parameter.Height.size(); // number of data points for column reference

		auto MaxRows = parameter.Max.rows();// number of row data points of table
		auto MaxCols = parameter.Max.cols();// number of column data points of table

		auto IdleRows = parameter.Idle.rows();// number of row data points of table
		auto IdleCols = parameter.Idle.cols();// number of column data points of table

		if ((IdleRows == MaxRows)&&(IdleCols == MaxCols)&& (IdleRows == Mach_index) && (IdleCols == Height_index)) {
			system_info.system_parameter_ok = 0;
			ready_to_run = true;
			MaxLookUp.LoadTableData(parameter.Mach, parameter.Height,parameter.Max,false);
			IdelLookUp.LoadTableData(parameter.Mach, parameter.Height,parameter.Idle,false);
		}
		else {
			system_info.system_parameter_ok = 1;
			ready_to_run = false;
		}
		// initialize state memeory
		state.resize(system_info.num_of_continuous_states);
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
	}

	void CFM56ThrustModel::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no differential equations for thrust model
	}

	void CFM56ThrustModel::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		Idle = IdelLookUp.GetOutput(input(CFM56_INPUT_Mach), input(CFM56_INPUT_Height));
		if (input(CFM56_INPUT_N1) < parameter.IdelN1) { // N1 < Idle N1
			output(0) = parameter.MaxThrust*input(CFM56_INPUT_N1) / parameter.IdelN1*Idle;
		}
		else { // N1> Idle N1
			Max = MaxLookUp.GetOutput(input(CFM56_INPUT_Mach), input(CFM56_INPUT_Height));
			output(0) = parameter.MaxThrust*((input(CFM56_INPUT_N1) - parameter.IdelN1) / N1_dff * (Max - Idle) + Idle);
		}

	}

	void CFM56ThrustModel::IncrementState()
	{
		// No increment state for thrust model
	}

	void CFM56ThrustModel::DisplayParameters()
	{
		if (system_info.system_parameter_ok == 0) {
			std::cout << "Mach Reference:  " << std::endl;
			std::cout << parameter.Mach << std::endl;
			std::cout << "Height Reference:  " << std::endl;
			std::cout << parameter.Height << std::endl;
			std::cout << "The Max table is:  " << std::endl;
			std::cout << parameter.Max << std::endl;
			std::cout << "The Idle table is:  " << std::endl;
			std::cout << parameter.Idle << std::endl;
			std::cout << "Max Thrust is :  " << std::endl;
			std::cout << parameter.MaxThrust << std::endl;
		}
		else {
			std::cout << "Input Table Dimension Mismatch!  " << std::endl;
		}
	}

	void CFM56ThrustModel::DisplayInitialCondition()
	{
		std::cout << "No initial condition for CFM 56 thrust model. " << std::endl;
	}

	CFM56ThrustModel::~CFM56ThrustModel() {

	}
	/* ---------------------------   propeller chart -------------------------------------------------*/
	PropellerChartFixedPitch::PropellerChartFixedPitch(const PropellerChartFixedPitchParameter & param)
	{
		parameter = param;
		// sets the system type and category 
		system_info.category = PROPULSION;
		system_info.type = propulsion_PROPELLERCHARTFIXEDPITCH;
		// set ready_to_run to true since there is not parameter check for this subsystem
		ready_to_run = true;
		// determine the size of the system
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 3;  
		system_info.num_of_outputs = 2; 
		// check the compatibility of the system matrices:
		system_info.system_parameter_ok = true;
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = true;
		// initialize state memeory
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
		// 0 th raw is J, 1 thrust, 2 power
		system_info.system_parameter_ok = 0;
		int num_of_cols = parameter.Chart.cols();
		propeller_fixed_pitch_.LoadTableData(parameter.Chart.col(0), parameter.Chart.rightCols(num_of_cols - 1), false);
		// initialize coefficient vector 
		Coefficient.resize(2, 1);
		Coefficient.setZero();
		D_4 = pow(parameter.diameter,4.0);
		D_5 = pow(parameter.diameter,5.0);
	}

	void PropellerChartFixedPitch::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no diferential equation for propeller chart
	}

	void PropellerChartFixedPitch::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		// 
		if (input(PROPELLER_INPUT_V) < parameter.minimumAngularRate) {
			output(PROPELLER_OUTPUT_Q) = 0.0;
			output(PROPELLER_OUTPUT_T) = 0.0;
		}
		else { // normal condition
			J = input(PROPELLER_INPUT_V) / (input(PROPELLER_INPUT_N)*parameter.diameter);
			N_2 = input(PROPELLER_INPUT_N) * input(PROPELLER_INPUT_N);
			propeller_fixed_pitch_.GetOutput(Coefficient,J); // use the first element of the input as the target height
			// T = rho n^2 D^4 CT, P = rho n^3 D_5 CP, Q = P/n = rho n^2 D_5 CP
			output(PROPELLER_OUTPUT_T) = input(PROPELLER_INPUT_RHO) * N_2 * D_4* Coefficient(0);// CT  = 0, CP = 1
			output(PROPELLER_OUTPUT_Q) = input(PROPELLER_INPUT_RHO) * N_2 * D_5* Coefficient(1);// CT  = 0, CP = 1
		}
	}

	void PropellerChartFixedPitch::IncrementState()
	{
		// no increment state for propeller chart block
	}

	void PropellerChartFixedPitch::DisplayParameters()
	{
		std::cout << "------ propeller chart parameter --------  \n";
		std::cout << " minimum angular rate is: " << parameter.minimumAngularRate << " (RPS) \n";
		std::cout << " propeller diameter is: " << parameter.diameter << " (m) \n";
		std::cout << " propeller chart is: \n";
		std::cout << parameter.Chart << "\n";
	}

	void PropellerChartFixedPitch::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for propeller chart block----------" << std::endl;
	}

	PropellerChartFixedPitch::~PropellerChartFixedPitch()
	{

	}

	/*---------------------- Varable pitch propeller --------------------------- */
	PropellerChartVariablePitch::PropellerChartVariablePitch(const PropellerChartVariablePitchParameter & param)
	{
		parameter = param;
		// sets the system type and category 
		system_info.category = PROPULSION;
		system_info.type = propulsion_PROPELLERCHARTVARIABLEPITCH;
		// determine the size of the system
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 4;
		system_info.num_of_outputs = 2;
		// check the compatibility of the system matrices:
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = true;

		// check the consistency of the thrust table:

		auto J_dimension = parameter.Jindex.size(); // number of data points for row reference
		auto Pitch_dimension = parameter.PitchIndex.size(); // number of data points for column reference

		auto ChartPRows = parameter.Chart_P.rows();// number of row data points of table
		auto ChartPCols = parameter.Chart_P.cols();// number of column data points of table

		auto ChartTRows = parameter.Chart_T.rows();// number of row data points of table
		auto ChartTCols = parameter.Chart_T.cols();// number of column data points of table

		if ((ChartPRows == ChartTRows) && (ChartPCols == ChartTCols) && (ChartTRows == J_dimension) && (ChartPCols == Pitch_dimension)) {
			system_info.system_parameter_ok = 0;
			ready_to_run = true;
			propeller_table_T_.LoadTableData(parameter.Jindex, parameter.PitchIndex, parameter.Chart_T, false);
			propeller_table_P_.LoadTableData(parameter.Jindex, parameter.PitchIndex, parameter.Chart_P, false);
		}
		else {
			system_info.system_parameter_ok = 1;
			ready_to_run = false;
		}
		// initialize state memeory
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
		D_4 = pow(parameter.diameter, 4.0);
		D_5 = pow(parameter.diameter, 5.0);
	}

	void PropellerChartVariablePitch::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
		// no diferential equation for propeller chart
	}

	void PropellerChartVariablePitch::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
		if (input(PROPELLER_INPUT_V) < parameter.minimumAngularRate) {
			output(PROPELLER_OUTPUT_Q) = 0.0;
			output(PROPELLER_OUTPUT_T) = 0.0;
		}
		else {
			N_2 = input(PROPELLER_INPUT_N) * input(PROPELLER_INPUT_N);
			J = input(PROPELLER_INPUT_V) / (input(PROPELLER_INPUT_N)*parameter.diameter);
			CT = propeller_table_T_.GetOutput(J, input(PROPELLER_INPUT_PITCH));
			CP = propeller_table_P_.GetOutput(J, input(PROPELLER_INPUT_PITCH));
			// T = rho n^2 D^4 CT, P = rho n^3 D_5 CP, Q = P/n = rho n^2 D_5 CP
			output(PROPELLER_OUTPUT_T) = input(PROPELLER_INPUT_RHO) * N_2 * D_4* CT;// CT  = 0, CP = 1
			output(PROPELLER_OUTPUT_Q) = input(PROPELLER_INPUT_RHO) * N_2 * D_5* CP;// CT  = 0, CP = 1
		}
	}

	void PropellerChartVariablePitch::IncrementState()
	{
		// no increment state
	}

	void PropellerChartVariablePitch::DisplayParameters()
	{
		std::cout << "------ propeller chart parameter --------  \n";
		std::cout << " minimum angular rate is: " << parameter.minimumAngularRate << " (RPS) \n";
		std::cout << " propeller diameter is: " << parameter.diameter << " (m) \n";
		std::cout << " propeller power chart is: \n";
		std::cout << parameter.Chart_P << "\n";
		std::cout << " propeller thrust chart is: \n";
		std::cout << parameter.Chart_T << "\n";
	}

	void PropellerChartVariablePitch::DisplayInitialCondition()
	{
		std::cout << "------No initial condition for propeller chart block----------" << std::endl;
	}

	PropellerChartVariablePitch::~PropellerChartVariablePitch()
	{
	}

	/*---------------------- pitson engine --------------------------- */

	PistonEngine::PistonEngine(const PistonEngineParameter & param)
	{
		parameter = param;
		// sets the system type and category 
		system_info.category = PROPULSION;
		system_info.type = propulsion_PISTONENGINE;
		// determine the size of the system
		system_info.num_of_continuous_states = 0;
		system_info.num_of_inputs = 4;
		system_info.num_of_outputs = 1; // shaft speed
		// check the compatibility of the system matrices:
		system_info.DIRECT_FEED_THROUGH = true;
		system_info.EXTERNAL_CONNECTION_ONLY = false;
		system_info.NO_CONTINUOUS_STATE = true;
		// initialize state

		// initialize state memeory
		output.resize(system_info.num_of_outputs);
		system_info.input_connection.resize(system_info.num_of_inputs, 2);
	}

	void PistonEngine::DifferentialEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & derivative)
	{
	}

	void PistonEngine::OutputEquation(const double & t, const VectorXd & state, const VectorXd & input, VectorXd & output)
	{
	}

	void PistonEngine::IncrementState()
	{
	}

	void PistonEngine::DisplayParameters()
	{
	}

	void PistonEngine::DisplayInitialCondition()
	{
	}

	PistonEngine::~PistonEngine()
	{
	}

}