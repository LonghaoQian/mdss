#pragma once
#include "Subsystem.h"
#define SQUARE 0
#define RAMP 1
#define SINE 2
#define STEP 3

namespace source_sink {
	struct SignalGeneratorparameter {
		double frequency;//HZ
		double amplitude;
		double phase_shift;
		int type;
		int num_of_channels;
	};

	using namespace std;

	class Source_SignalGenerator :
		public Subsystem
	{
	private:
		SignalGeneratorparameter parameter;
		double omega;// angular velocity
	public:
		Source_SignalGenerator(const SignalGeneratorparameter& _parameter);
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& temp_derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Source_SignalGenerator();
	};

}