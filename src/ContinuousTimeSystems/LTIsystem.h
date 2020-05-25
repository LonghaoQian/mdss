#pragma once
#include "Subsystem.h"

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
}



