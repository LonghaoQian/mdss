#pragma once
#include "RungeKuttaFamily.h"
#include "Subsystem.h"
#include "TopologyAnalysis.h"
// Include all subsystems
#include "LinearSystem.h"
#include "SignalGenerator.h"
#include "RigidBody.h"
#include "StandardAtmosphere.h"
#include "MathBlocks.h"
#include "AeroAngle.h"
#include "AeroForceMoment.h"
#include "DiscontinuousSystem.h"
#include "PropulsionBlocks.h"

using namespace Eigen;
using namespace std;
static const std::map<subsystem_type, std::string> subsystem_type_list{
	{aero_AROANGLE, "Aero angles block"},
	{aero_AROFORCEMENT, "Aeroforce block"},
	{continous_INTEGRATOR, "Integrator block"},
	{continous_LTI, "LTI"},
	{continous_PIDcontroller, "PID-controller block"},
	{continous_RIGIDBODY,"Rigid-body block"},
	{continous_RIGIDDYNAMICS, "Rigid-dynamics block"},
	{continous_RIGIDKINEMATICS, "Rigd-Kinematics block"},
	{continous_TRANSFERFUNCTION, "Transfer-function block"},
	{continous_VARIABLEMASS, "Variable mass body"},
	{discontinuous_SATURATION, "Saturation block"},
	{discontinuous_SWITCH, "Switch block"},
	{geographic_ATOMSPHERE, "Standard atmopshere "},
	{geographic_GRAVITY, "Gravity block"},
	{math_CONSTANT, "Constant block"},
	{math_CROSSPRODUCT, "Cross-product block"},
	{math_GAIN, "Gain block "},
	{math_LOOKUP1D,"1D Lookup block"},
	{math_LOOKUP2D,"2D Lookup block"},
	{math_PRODUCT, "Product block"},
	{math_SPECIALFUNCTION,"Special-function block"},
	{math_TRIGONOMETRYFUNCTION,"Trignometry-function block"},
	{math_SUM, "Summation block"},
	{propulsion_CFM56AUXILIARYMODEL, "CFM56 Auxiliary Model block"},
	{propulsion_CFM56THRUST, "CFM56 Thrust block"},
	{propulsion_PROPELLERCHARTFIXEDPITCH, "Propeller chart fixed pitch"},
	{propulsion_PROPELLERCHARTVARIABLEPITCH, "Propeller chart variable pitch" },
	{propulsion_PISTONENGINE,"Piston engine"},
	{source_SINGALGENERATOR, "Signal Generator "},
	{source_STEP, "Step block"},
	{source_RAMP, "Ramp block"}
};
namespace simulationcontrol {
	typedef MatrixX2i SIMCONNECTION;
	typedef vector<subsystem_handle>::iterator subsystemhandlePtr;

	enum LogMsgLevel { // log level
		LOGLEVEL_NONE = 0,    // disp nothing
		LOGLEVEL_ERROR,   // disp errors
		LOGLEVEL_WARN,    // disp warnings and errors
		LOGLEVEL_ALL, // disp all msg	
	};

	enum signalrouting
	{
		external = -1,
		subsystemID = 0,
		outputportID = 1,
	};

	struct DataLogging {
		bool uselogging; // flag determine whether logging is on
		string filename; // file name 
	};

	struct LoggerTag{
		string tag;
		unsigned int output_system_ID;
		unsigned int output_port_ID;
	};

	struct SolverConfig {
		double mim_step;
		double frame_step;// step_size according to the frame
		RungeKuttaFamily::SolverType solver_type;
		double eposilon;
		unsigned int num_of_k;
		bool adaptive_step;
		double start_time;
		DataLogging loggingconfig;
		LogMsgLevel loglevel{ LOGLEVEL_ALL };
	};

	class SimController
	{
	private:
		bool system_ok;// if all good, the system is ok and ready to go.
		//-------------------------- system parameters---------------------------------//
		unsigned int num_of_subsystems;
		unsigned int num_of_continuous_states;
		unsigned int num_of_outputs;
		unsigned int num_of_external_inputs;
		SolverConfig solver_config;
		vector<unique_ptr<Subsystem>> subsystem_list;// a list of all subsystems
		MatrixXi connectivity;// connectivity map of the simulation
		MatrixXi external_mapping;
		LogMsgLevel loglevel{ LOGLEVEL_ALL };
		// temp space for numerical integration
		//----------------------------- Solver Variables---------------------------//
		bool GetExternalInputs(const VectorXd& extern_input);// buffer the external inputs
		MatrixXd butchertableau;
		double current_stepsize;
		double current_time;
		int num_of_steps_per_cycle;
		/*--------------------------------System Topology------------------------------*/
		vector<bool> temp_all_susystems;// the a list of whether the subsystem is ready for output update
		vector<int> output_sequence;
		vector<int> non_direct_feedthrough_index;
		vector<VectorXi> algebraric_loops;
		VectorXd step_error;// step error for each continuous system
		VectorXd updatecoefficient1;
		VectorXd updatecoefficient2;
		int num_of_cycles_per_step;
		int num_of_closed_loops;
		int number_of_not_ready;// number of not ready subsystems for output sequence
		bool DetermineOutputSequenceDFS(int level, unsigned int base_index);
		bool RunTopologyAnalysis();
		string GetSystemTypeFromID(subsystem_type type);
		unsigned int CreateSystemHandle(const subsystem_info& info, const vector<unique_ptr<Subsystem>>& subsystem_list);// subsystem handle
		vector<unique_ptr<subsystem_handle>> system_handle_list;// the list of handles
		Matrix<bool, Dynamic, 1> connection_flag_list;
		/*--------------------- Data logging -------------------------------------------*/
		ofstream loggingdata;
		std::vector<Matrix<int, 1, 2>> logportlist; // list containing the mapping of the subsystemID and map
		std::vector<std::string> logtaglist;
		int total_number_log = 0;
		void LogRequestedData();
	public:
		/* A list of all overloadings of AddSubsystem(...) for pre-defined types of models*/
		// Linear system
		unsigned int AddSubSystem(const linearsystem::LTIParameter& parameters, const linearsystem::LTIInitialCondition& IC);
		unsigned int AddSubSystem(const linearsystem::IntegratorParameter& parameters, const linearsystem::IntegratorInitialCondition& IC);
		unsigned int AddSubSystem(const linearsystem::TransferFunctionParameter& parameters);
		unsigned int AddSubSystem(const linearsystem::PIDcontrollerParameter& parameters);
		unsigned int AddSubSystem(const linearsystem::RateLimitedActuatorParameter& parameters, const linearsystem::RateLimitedActuatorInitialCondition& IC);
		// Discontinuous
		unsigned int AddSubSystem(const discontinuoussystem::SaturationParameter& parameters);
		unsigned int AddSubSystem(const discontinuoussystem::SwitchParameter& parameters);
		// Dynamics 
		unsigned int AddSubSystem(const dynamics::RigidBodyParameter& parameters, const dynamics::RigidBodyCondition& IC);
		unsigned int AddSubSystem(const dynamics::RigidBodyKinematicsInitialCondition& IC);
		unsigned int AddSubSystem(const dynamics::RigidBodyDynamicsParamter& parameters);
		// math blocks
		unsigned int AddSubSystem(const mathblocks::GainParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::ConstantParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::SumParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::MultiplicationParam& parameters);
		unsigned int AddSubSystem(const mathblocks::TrigonometryParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::Lookup1DParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::Lookup2DParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::SpecialFunctionParameter& parameters);
		unsigned int AddSubSystem(const mathblocks::CrossProductParameter& param);
		// geographic libs
		unsigned int AddSubSystem(const geographic::StandardAtmosphereParameter& parameters);
		unsigned int AddSubSystem(const geographic::GravityModelParameter& parameters);
		// aerodynamics
		unsigned int AddSubSystem(const aero::AerosForceParameter& parameters);
		unsigned int AddSubSystem(const aero::AeroAngleParameter& parameters);
		// source and sinks
		unsigned int AddSubSystem(const  source_sink::PeriodicWaveparameter& parameters);
		unsigned int AddSubSystem(const  source_sink::Stepparameter& parameters);
		unsigned int AddSubSystem(const  source_sink::Rampparamter& parameters);
		// propulsion
		unsigned int AddSubSystem(const  propulsionsystem::CFM56Parameter& parameters);
		unsigned int AddSubSystem(const  propulsionsystem::CF56ThrustModelParameter& parameters);
		// TO DO: utility blocks
		// unit conversion, matrix to euler angles, matrix quaternion, quaternion euler angle
		/*------------------------define connections between subsystems--------------------------------*/
		bool ResetInitialCondition(); // TO DO: reset subsystem initial condition
		void EditConnectionMatrix(unsigned int handleID,
								  unsigned int from_input_ID, 
								  unsigned int to_output_systemID, 
								  unsigned int to_output_portID);
		void BatchEditConnectionMatrix(unsigned int input_system_ID,
									   unsigned int input_portID_start,
									   unsigned int input_portID_end,
									   unsigned int output_system_ID,
									   unsigned int output_port_ID_start,
									   unsigned int output_port_ID_end);
		bool MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping);// batch connection
		bool MakeConnection(const subsystem_handle& handle);
		bool FlushMakeConnection();
		subsystem_handle GetSystemHandle(const unsigned int system_ID);
		/*------------------------PreRunProcess of the Connected Subystems--------------*/
		void EditSolverConfig(const SolverConfig& config);
		bool PreRunProcess();// check and parse the system connection relationship.
		void DisplayTopology();
		void ReshapeExternalInputVector(VectorXd& extern_input);
		void DisplaySystemParameter(unsigned int system_ID);
		void DisplaySystemInitialCondition(unsigned int system_ID);
		/*------------------------Run Time Function -------------------------------------*/
		int Run_Update(const VectorXd& extern_input);
		double Run_GetSystemTime();
		VectorXd Run_GetSubsystemOuput(const unsigned int system_ID);
		/*------------------------Data logging--------------------------------------*/
		bool DefineDataLogging(const unsigned int output_system_ID,
							   const unsigned int output_port_ID,
							   string tag);
		LoggerTag GetLoggerTag(unsigned int TagIndex);
		void DisplayLoggerTagList();
		/*-----------------------Post run process----------------------------------------*/
		int PostRunProcess();
		/*-----------------------Solver definition--------------------------------------*/
		SimController(const SolverConfig& config);
		SimController();// default constructor
		~SimController();
	};
}
