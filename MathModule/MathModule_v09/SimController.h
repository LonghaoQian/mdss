#pragma once
#include <Eigen\Dense>
#include "RungeKuttaFamily.h"
#include "Subsystem.h"
#include "LTIsystem.h"
#include <iostream>
#include "RigidBody.h"
#include "TopologyAnalysis.h"
using namespace Eigen;
using namespace std;
struct SolverConfig {
	double max_step;        
	double mim_step;
	int solver_type;
	double eposilon;
	unsigned int num_of_k;
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
	/*--------------------------------System Topology------------------------------*/
	vector<int> output_sequence;
	vector<VectorXi> algebraric_loops;
	int num_of_closed_loops;
	bool RunTopologyAnalysis();
public:
	/*overloads of AddSubsystems to suit for every pre-defined type of model*/
	bool AddSubSystem(const LTIParameter& parameters, const  LTIInitialCondition& IC); // input all system info to the sim control object
	bool AddSubSystem(const RigidBodyParameter& parameters, const RigidBodyCondition& IC); // input all system info to the sim control object
	/*------------------------define connections between subsystems--------------------------------*/
	bool MakeConnection(unsigned int system_ID, const MatrixX2i& connection_mapping);
	/*------------------------PreRunProcess of the Connected Subystems--------------*/
	bool PreRunProcess();// check and parse the system connection relationship.
	int Run(const double& t, const VectorXd& extern_input);
	int PostRunProcess();
	void DisplayTopology();
	SimController(const SolverConfig& config);
	~SimController();
};

