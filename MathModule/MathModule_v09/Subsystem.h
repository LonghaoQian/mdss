#pragma once
#include <Eigen\Dense>
#include <vector>
// connection type
#define NOCONNECTION -1
#define EXTERNAL -2
// 
#define GENERIC 1
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
	unsigned int subsystem_ID;// a unique ID for all subsystems
	unsigned int subsystem_type;// system type
	MatrixX2i input_connection;
};
class Subsystem
{
protected:
	bool ready_to_run;
	subsystem_info system_info;
	// state and output of the subsystem at a given time instance
	VectorXd state;
	VectorXd output;
	// connection property
public:
	virtual void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& temp_derivative) = 0;// differential equation for the system
	virtual void OutputEquation(const VectorXd& state, const VectorXd& input, VectorXd& output)=0;// output of the sub system
	virtual void LoadInitialCondition(const VectorXd& initial_condition) = 0;
	virtual void LoadParameters(const VectorXd& parameter_list) = 0;
	void SetInputConnection(const MatrixX2i& connection);
	subsystem_info GetSystemInfo();
	Subsystem();
	~Subsystem();
};

