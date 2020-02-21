#pragma once
#include "RungeKuttaFamily.h"
#include "Subsystem.h"
#include "TopologyAnalysis.h"
// Include all subsystems
#include "LTIsystem.h"
#include "Source_SignalGenerator.h"
#include "RigidBody.h"
#include "StandardAtmosphere.h"
#include "MathBlocks.h"
#include "AeroAngle.h"
#include "AeroForceMoment1.h"
#define EXTERNAL_INPUT -1
#define UNCONNECTED -2
/* TO DO: 
1. if external is not defined, keep them as -2 as unconnected and set to 0
2. makeconnection ( from_ouput_ID, to_input,ID, )
3. create group ()
*/
using namespace Eigen;
using namespace std;
typedef MatrixX2i SIMCONNECTION;
struct SolverConfig {      
	double mim_step;
	double frame_step;// step_size according to the frame
	int solver_type;
	double eposilon;
	unsigned int num_of_k;
	bool adaptive_step;
	double start_time;
};

struct SubsystemGroupHandle {
	int num_of_inputs;
	int num_of_ouputs;
	int num_of_blocks;


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
	// temp space for numerical integration
	//----------------------------- Solver Variables---------------------------//
	bool GetExternalInputs(const VectorXd& extern_input);// buffer the external inputs
	MatrixXd butchertableau;
	double current_stepsize;
	double current_time;
	int num_of_steps_per_cycle;
	/*--------------------------------System Topology------------------------------*/
	vector<int> output_sequence;
	vector<int> non_direct_feedthrough_index;
	vector<VectorXi> algebraric_loops;
	VectorXd step_error;// step error for each continuous system
	VectorXd updatecoefficient1;
	VectorXd updatecoefficient2;
	int num_of_cycles_per_step;
	int num_of_closed_loops;
	bool RunTopologyAnalysis();
	string GetSystemTypeFromID(int sys_ID_);
	subsystem_handle CreateSystemHandle(const subsystem_info& info, const vector<unique_ptr<Subsystem>>& subsystem_list);
	// TO DO: add parse system group function to preprocess
public:
	/*overloads of AddSubsystems to suit for every pre-defined type of model*/
	// continuous state
	subsystem_handle AddSubSystem(const LTIParameter& parameters, const  LTIInitialCondition& IC); // input all system info to the sim control object
	subsystem_handle AddSubSystem(const RigidBodyParameter& parameters, const RigidBodyCondition& IC); // input all system info to the sim control object
	// math blocks
	subsystem_handle AddSubSystem(const mathblocks::GainParameter& parameters);
	subsystem_handle AddSubSystem(const mathblocks::ConstantParameter& parameters);
	subsystem_handle AddSubSystem(const mathblocks::SumParameter& parameters);
	subsystem_handle AddSubSystem(const mathblocks::MultiplicationParam& parameters);
	// geographic libs
	subsystem_handle AddSubSystem(const geographic::StandardAtmosphereParameter& parameters);
	subsystem_handle AddSubSystem(const geographic::GravityModelParameter& parameters);
	// aerodynamics
	subsystem_handle AddSubSystem(const aero::AerosForceParameter& parameters);
	subsystem_handle AddSubSystem(const aero::AeroAngleParameter& parameters);
	// source and sinks
	subsystem_handle AddSubSystem(const  source_sink::SignalGeneratorparameter& parameters);
	// add subsystem assembly
	//bool DefineSubSystemAssembly();
	// TO DO: add special block creation functions
	// subsystem_handle AddEngineSystem();
	/*------------------------define connections between subsystems--------------------------------*/
	bool MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping);
	/*------------------------PreRunProcess of the Connected Subystems--------------*/
	bool PreRunProcess();// check and parse the system connection relationship.
	void DisplayTopology();
	/*------------------------Run Time Function -------------------------------------*/
	int Run_Update(const VectorXd& extern_input);
	double Run_GetSystemTime();
	VectorXd Run_GetSubsystemOuput(const unsigned int system_ID);
	/*-----------------------Post run process----------------------------------------*/
	int PostRunProcess();
	SimController(const SolverConfig& config);
	~SimController();
};

