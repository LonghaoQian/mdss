#pragma once
// connection type
#define NOCONNECTION -1
#define EXTERNAL -2
// A template for a subsystem used for schecduling.
using std::vector;
using namespace Eigen;
typedef Matrix<int, Dynamic, 2> MatrixX2i;

enum subsystem_category {
	AERODYNAMICS = 0,
	LINEARSYSTEM,
	DISCONTINUOUS,
	DYNAMICS,
	GEOGRAPHIC,
	MATH,
	PROPULSION,
	SOURCE,
	UTILITY,
};

enum subsystem_type {
	aero_AROANGLE,
	aero_AROFORCEMENT_1,
	continous_INTEGRATOR,
	continous_TRANSFERFUNCTION,
	continous_LTI,
	continous_PIDcontroller,
	continous_RIGIDBODY,
	continous_RIGIDDYNAMICS,
	continous_RIGIDKINEMATICS,
	continous_VARIABLEMASS,
	discontinuous_SATURATION,
	discontinuous_SWITCH,
	geographic_ATOMSPHERE,
	geographic_GRAVITY,
	math_CONSTANT,
	math_CROSSPRODUCT,
	math_GAIN,
	math_LOOKUP1D,
	math_LOOKUP2D,
	math_PRODUCT,
	math_SPECIALFUNCTION,
	math_SUM,
	math_TRIGONOMETRYFUNCTION,
	propulsion_CFM56AUXILIARYMODEL,
	propulsion_CFM56THRUST,
	source_SINGALGENERATOR,
	source_STEP,
	source_RAMP
};

static const std::map<subsystem_type, std::string> subsystems[7];


static const std::map<subsystem_type, std::string> subsystem_type_list{
	{aero_AROANGLE, "Aero angles block"},
	{aero_AROFORCEMENT_1, "Aerodynamics Forces and Moments"},
	{continous_INTEGRATOR, "Integrator"},
	{continous_LTI, "LTI"},
	{continous_PIDcontroller, "PID controller"},
	{continous_RIGIDBODY,"Rigid body"},
	{continous_VARIABLEMASS, "Variable mass body"},
	{discontinuous_SATURATION, "Saturation"},
	{discontinuous_SWITCH, "Switch"},
	{geographic_ATOMSPHERE, "Standard atmopshere "},
	{geographic_GRAVITY, "Gravity block"},
	{math_CONSTANT, "Constant block"},
	{math_CROSSPRODUCT, "Cross product block"},
	{math_GAIN, "Gain block "},
	{math_LOOKUP1D,"1D Lookup block"},
	{math_LOOKUP2D,"2D Lookup block"},
	{math_PRODUCT, "Product block"},
	{math_SPECIALFUNCTION,"Special function block"},
	{math_TRIGONOMETRYFUNCTION,"Special function block"},
	{math_SUM, "Summation block"},
	{propulsion_CFM56AUXILIARYMODEL, "CFM56 Auxiliary Model block"},
	{propulsion_CFM56THRUST, "CFM56 Thrust block"},
	{source_SINGALGENERATOR, "Signal Generator "},
	{source_STEP, "Step block"},
	{source_RAMP, "Ramp block"}
};
struct subsystem_handle {
	subsystem_type type;
	int ID;			// the index this subsystem is assigned after the addsubsystem function
	std::string label_;
	bool isParameterOK;
	MatrixX2i input_connection_list;
};
// subsystem info for the solver TO DO: add a mandatory function 
struct subsystem_info {
	unsigned int num_of_continuous_states; // number of continunous states, for direct feed-through blocks, this should be 0
	unsigned int num_of_inputs;            // number of inputs for the system
	unsigned int num_of_external_inputs;   // number of external inputs for the system
	unsigned int num_of_outputs;           // number of outputs for the system
	bool system_parameter_ok;              // wether the system parameter has been correctly set
	MatrixX2i input_connection;            // the input connection matrix, where each row represents 
	bool NO_CONTINUOUS_STATE;              // a bool state repes wether continous state exsits
	bool DIRECT_FEED_THROUGH;              // a bool state 
	bool EXTERNAL_CONNECTION_ONLY;         // a bool state for whether the system connection is pure external
	subsystem_type type;                   // the type of the system, defined in subsystem_type
	subsystem_category category;            // the category of the system. 
	std::string label_;	                   // an optional name given to the subsystem. 
	std::map<std::string, unsigned int> input_label; // an optional map to give each input a name
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
	// relative error for adaptive step
	VectorXd relative_error;
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

