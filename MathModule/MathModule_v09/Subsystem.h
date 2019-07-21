#pragma once
#include <Eigen\Dense>
#include <vector>
// connection type
#define NOCONNECTION -1
#define EXTERNAL -2
// 
#define GENERIC 0
#define LTI 1
#define INTEGRATOR 2
#define RIGIDBODY 3
// A template for a subsystem used for schecduling.
using namespace std;
using namespace Eigen;
typedef Matrix<int, Dynamic, 2> MatrixX2i;
struct subsystem_info {
	unsigned int num_of_continuous_states;
	unsigned int num_of_inputs;
	unsigned int num_of_outputs;
	unsigned int system_type;
	bool system_parameter_ok;
	MatrixX2i input_connection;
	bool NO_CONTINUOUS_STATE;
	bool DIRECT_FEED_THROUGH;
};
class Subsystem
{
protected:
	bool ready_to_run;
	subsystem_info system_info;
	// state and output of the subsystem at a given time instance
	VectorXd state;
	VectorXd output;
	// temporary output for numerical solver
	VectorXd solver_buffer_output_temp;
	VectorXd solver_buffer_input_temp;
	VectorXd solver_buffer_state_temp;
	VectorXd solver_buffer_Ki_temp;
	// k1,...,kn vector for numerical solver
	MatrixXd solver_buffer_k_sequence;
public:
	/*----------SystemSetUp--------------*/
	virtual void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& temp_derivative) = 0;// differential equation for the system
	virtual void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output)=0;// output of the sub system
	virtual void IncrementState(const VectorXd& state_increment) = 0;
	virtual VectorXd GetState() = 0;
	virtual void UpdateOutput(const double& t, const VectorXd& input) = 0;
	virtual VectorXd GetOutput() = 0;
	virtual void DisplayParameters() = 0;
	virtual void DisplayInitialCondition() = 0;
	void SetInputConnection(const MatrixX2i& connection);
	subsystem_info GetSystemInfo();
	/**-----------SolverRelated--------*/
	void Solver_InitSolverBuffer(unsigned int num_of_kn);// Solver Init Function
	void Solver_UpdateKiBuffer(int index, double& current_time, double& stepsize, const MatrixXd& butchertableau);
	void Solver_UpdateInputTemp(int index, double input_temp_i);
	void Solver_PreturbState(int index, const MatrixXd& butchertableau);
	VectorXd Solver_GetInputTemp();
    void Solver_PreturbOutput(int index, double& current_time, double& stepsize, const MatrixXd& butchertableau);
	Subsystem();
	~Subsystem();
};

