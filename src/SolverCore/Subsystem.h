#pragma once
// connection type
#define NOCONNECTION -1
#define EXTERNAL -2
// TO DO: better indexing for subsystems
#define GENERIC			 0x00
#define LTI				 0x01
#define INTEGRATOR		 0x02
#define RIGIDBODY		 0x03
#define VARIABLMASS_0    0x04
#define GAIN			 0x10
#define SATURATION       0x11
#define CONSTANT         0x12
#define MULTIPLICATION   0x13
#define SUM              0x14
#define Signal_Generator 0x20
#define ATOMSPHERE		 0x30
#define GRAVITY          0x31
#define AROANGLE         0x41
#define AeroForceMoment  0x42
// A template for a subsystem used for schecduling.
using std::vector;
using namespace Eigen;
typedef Matrix<int, Dynamic, 2> MatrixX2i;

enum subsystem_type {
	// alphabic order
	system_group, // system group is a special type of subsystem
	aero_AROANGLE,
	aero_AROFORCEMENT_1,
	continous_INTEGRATOR,
	continous_LTI,
	continous_RIGIDBODY,
	continous_VARIABLEMASS,
	geographic_ATOMSPHERE,
	geographic_GRAVITY,
	math_CONSTANT,
	math_CROSSPRODUCT,
	math_GAIN,
	math_PRODUCT,
	math_SATURATION,
	math_SUM,
	source_SINGALGENERATOR,
};

struct subsystem_handle {
	subsystem_type type;
	int Layer;// determine 
	int ID;
	std::string label_;
	bool isParameterOK;
};
// subsystem info for the solver
struct subsystem_info {
	unsigned int num_of_continuous_states;
	unsigned int num_of_inputs;
	unsigned int num_of_external_inputs;
	unsigned int num_of_outputs;
	unsigned int system_type;
	bool system_parameter_ok;
	MatrixX2i input_connection;
	bool NO_CONTINUOUS_STATE;
	bool DIRECT_FEED_THROUGH;
	subsystem_type type;
	std::string label_;
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
	VectorXd solver_buffer_state_increment1;
	VectorXd solver_buffer_state_increment2;
	// k1,...,kn vector for numerical solver
	MatrixXd solver_buffer_k_sequence;
public:
	/*----------SystemSetUp--------------*/
	virtual void DifferentialEquation(const double& t, 
									  const VectorXd& state,
									  const VectorXd& input,
									  VectorXd& temp_derivative) = 0;// differential equation for the system
	virtual void OutputEquation(const double& t,
								const VectorXd& state, 
								const VectorXd& input, 
								VectorXd& output)=0;// output of the sub system
	virtual void IncrementState() = 0;
	virtual void DisplayParameters() = 0;
	virtual void DisplayInitialCondition() = 0;
	/**------------------------------------*/
	VectorXd GetState();
    void UpdateOutput(const double& t, const double& current_stepsize);
	VectorXd GetOutput();
	void SetInputConnection(const MatrixX2i& connection);
	void OverrideDirectFeedThroughFlag(bool isDirectFeedThrough);
	subsystem_info GetSystemInfo();
	/**-----------SolverRelated--------*/
	void Solver_InitSolverBuffer(unsigned int num_of_kn);// Solver Init Function
	void Solver_UpdateKiBuffer(int index, 
							   double& current_time, 
							   double& stepsize, 
							   const MatrixXd& butchertableau);
	void Solver_UpdateInputTemp(int index, double input_temp_i);
	void Solver_PreturbState(int index, const MatrixXd& butchertableau);
	VectorXd Solver_GetOuputTemp();
	void Solver_CalculateIncrement(const VectorXd& updatecoefficients);
	double Solver_CalculateIncrement(const VectorXd& updatecoefficients1, 
									 const VectorXd& updatecoefficients2);// two coefficiet overload
	VectorXd Solver_GetInputTemp();
    void Solver_PreturbOutput(int index,
							  double& current_time, 
							  double& stepsize, 
							  const MatrixXd& butchertableau);
	Subsystem();
	~Subsystem();
};

