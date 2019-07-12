#pragma once
#include <Eigen\Dense>
#include "Subsystem.h"
#include "LTIsystem.h"
using namespace Eigen;
using namespace std;
struct SolverConfig {
	double max_step;        
	double mim_step;
	int solver_type;
	double eposilon;
};
class SimController
{
private:
	bool system_ok;// if all good, the system is ok and ready to go.
	unsigned int num_of_subsystems;
	unsigned int num_of_continuous_states;
	unsigned int num_of_outputs;
	unsigned int num_of_external_inputs;
	SolverConfig solver_config;
	vector<unique_ptr<Subsystem>> subsystem_list;// a list of all subsystems
	vector<unique_ptr<VectorXd>> external_input_list;// a list of all external inputs
	vector<unique_ptr<MatrixX2i>> output_connection;// a list of all output_connections
	vector<unique_ptr<MatrixX2i>> input_connection;// al iist of all input connections
	MatrixXd connectivity;// connectivity map of the simulation
	// temp space for numerical integration
	void UpdateExternalInputs(const VectorXd& external_input);// get external inputs
	int Add_subsystem(unsigned  int system_type, const MatrixX2i& input_connection, const VectorXd& parameters); // input all system info to the sim control object
	int LoadInitialCondition();
	int PreRunProcess();// Parsing all the subsystems, load parameters, and check the con
public:
	int Run();
	int PostRunProcess();
	int GetSystemInfo();
	SimController(const SolverConfig& config, const vector<subsystem_info>& system_construct, const vector<VectorXd>& parameter_list);
	~SimController();
};

