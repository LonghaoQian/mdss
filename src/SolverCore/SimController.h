#pragma once

#include <Eigen\Dense>
#include "RungeKuttaFamily.h"
#include "Subsystem.h"
#include "LTIsystem.h"
#include "Math_Gain.h"
#include "Source_SignalGenerator.h"
#include "RigidBody.h"
#include "TopologyAnalysis.h"
#include "StandardAtmosphere.h"

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
public:
	/*overloads of AddSubsystems to suit for every pre-defined type of model*/
	bool AddSubSystem(const LTIParameter& parameters, const  LTIInitialCondition& IC); // input all system info to the sim control object
	bool AddSubSystem(const RigidBodyParameter& parameters, const RigidBodyCondition& IC); // input all system info to the sim control object
	bool AddSubSystem(const Gainparameter& parameters);
	bool AddSubsystem(const SignalGeneratorparameter& parameters);
	bool AddSubsystem(const StandardAtmosphereParameter& parameters);
	//bool AddSUbSystem(const GainParameter& parameters);
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

