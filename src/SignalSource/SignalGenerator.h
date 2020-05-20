// generates periodic wave shapes as source
#pragma once
#include "Subsystem.h"

namespace source_sink {
	enum shape {
		SQUARE,
		TRANGLE,
		SINE,
	};
	// name list
static const std::map<shape,std::string> wavename{
			{SQUARE, "Square"},
			{TRANGLE,"Trangle"},
			{SINE,"Sine"},
	};
	struct PeriodicWaveparameter {
		double frequency;//HZ
		double amplitude;
		double phase_shift;
		shape  waveshape;
		int num_of_channels;
	};

	class PeriodicWave :
		public Subsystem
	{
	private:
		PeriodicWaveparameter parameter;
		double omega;// angular velocity
		double temp; // a storage for the temp sine wave
	public:
		PeriodicWave(const PeriodicWaveparameter& _parameter);
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
		~PeriodicWave();
	};

	struct Stepparameter
	{
		double steptime;
		double value;
		int num_of_channels;
	};

	class Step :
		public Subsystem 
	{
	private:
		Stepparameter parameter;
	public:
		Step(const Stepparameter& _parameter);
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
		~Step();
	};

	struct Rampparamter
	{
		double starttime;
		double rate;
		int num_of_channels;
	};
	class Ramp :
		public Subsystem
	{
	private:
		Rampparamter parameter;
	public:
		Ramp(const Rampparamter& parameter_);
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
		~Ramp();
	};
}