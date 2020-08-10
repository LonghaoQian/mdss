#pragma once
#include "Subsystem.h"
/*
_________________________________
Author: Longhao Qian
Data:   2020 07 19

linearsystem contains the following blocks:

1. LTI system block
2. Integrator block
3. TransferFunction block
4. RateLimitedActuator block
5. PID block
_________________________________
*/
namespace linearsystem {
	struct LTIParameter{
		MatrixXd A;
		MatrixXd B;
		MatrixXd C; 
		MatrixXd D;
	};
	struct LTIInitialCondition {
		VectorXd X_0;
	};
	using namespace std;
	class LTIsystem :
		public Subsystem
	{
	private:
		// parameters
		MatrixXd A, B, C, D;
	public:
		LTIsystem();
		LTIsystem(const LTIParameter& parameter, const LTIInitialCondition& IC);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~LTIsystem();
	};

	struct IntegratorParameter {
		int num_of_channels;
	};
	struct IntegratorInitialCondition {
		VectorXd X_0;
	};
	class Integrator :
		public Subsystem
	{
	private:
		IntegratorParameter parameter;
	public:
		Integrator();
		Integrator(const IntegratorParameter& parameter, const IntegratorInitialCondition& IC);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Integrator();
	};

	struct TransferFunctionParameter {
		VectorXd Numerator; 
		VectorXd Denominator;
	};
	class TransferFunction :
		public Subsystem
	{
	private:
		TransferFunctionParameter parameter;
		MatrixXd A, B, C, D;
		void SystemMatricesFormStrictlyProperTF(const VectorXd& Numerator, const VectorXd& Denomiator);
	public:
		TransferFunction();
		TransferFunction(const TransferFunctionParameter& parameter);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~TransferFunction();
	};

	struct RateLimitedActuatorParameter {
		double maximum_rate;
		double time_constant;
		int num_of_channels;
	};

	class RateLimitedActuator :
		public Subsystem
	{
	private:
		RateLimitedActuatorParameter param_;
	public:
		RateLimitedActuator();
		RateLimitedActuator(const RateLimitedActuatorParameter& parameter);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~RateLimitedActuator();
	};
	// TO DO; add another mode where the integration is done normally
	struct PIDcontrollerParameter {
		double Kp;
		double Ki;
		double Kd;
		double Tf;
		int num_of_channels;
		bool integration_control_on;// if this is true, the first input is a switch to enable the controller. mode is reset
	};

	struct PIDcontrollerInitialCondition{
		VectorXd initial_integrator;
	};

	class PIDcontroller :
		public Subsystem
	{
	private:
		PIDcontrollerParameter param_;
		Matrix<double, 2, 2> A;
		Matrix<double, 2, 1> B;
		Matrix<double, 1, 2> C;
		Matrix<double, 1, 1> D;
	public:
		PIDcontroller(const PIDcontrollerParameter& parameter);
		void DifferentialEquation(const double& t, 
								  const VectorXd& state, 
								  const VectorXd& input, 
								  VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~PIDcontroller();
	};
}



