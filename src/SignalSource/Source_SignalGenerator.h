// generates periodic wave shapes as source
#pragma once
#include "Subsystem.h"

namespace source_sink {
	enum shape {
		SQUARE,
		TRANGLE,
		SINE,
		STEP
	};
	// name list
static const std::map<shape,std::string> wavename{
			{SQUARE, "Square"},
			{TRANGLE,"Trangle"},
			{SINE,"Sine"},
			{STEP,"Step"}
	};
	struct SignalGeneratorparameter {
		double frequency;//HZ
		double amplitude;
		double phase_shift;
		shape  waveshape;
		int num_of_channels;
	};

	class Source_SignalGenerator :
		public Subsystem
	{
	private:
		SignalGeneratorparameter parameter;
		double omega;// angular velocity
		double temp; // a storage for the temp sine wave
	public:
		Source_SignalGenerator(const SignalGeneratorparameter& _parameter);
		// the differential equation is left empty
		void DifferentialEquation(const double& t, 
								  const VectorXd& state, 
								  const VectorXd& input, 
								  VectorXd& temp_derivative);
		void OutputEquation(const double& t, 
							const VectorXd& state, 
							const VectorXd& input,
							VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~Source_SignalGenerator();
	};

}