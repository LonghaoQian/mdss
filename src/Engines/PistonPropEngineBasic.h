#pragma once
/*
-------------------------------------------------------
Author: Longhao Qian 2020 Jan 6th
Subsystem for simple piston prop engine thrust
Input:
0.throttle
1.TAS
2.density
3.starter
4.mixture
5.prop pitch command
6.p
7.q
8.r
9 alpha
10 beta

State:
0. n
1. prop_position

Output:
0. RPM
1-3.T
4-6.Mt
7. Power
8. fuelrate
9. engine_state
-------------------------------------------------------
*/
#include "Subsystem.h"
namespace engine {
	struct PistonEngineBasicParam {
		Vector3d n_prop_;
		Vector3d engine_position_;// engine position from reference point
		MatrixXd CP;
		MatrixXd CT;
		VectorXd J_reference_;
		VectorXd prop_pitch_reference_;
		VectorXd engine_power_data_;
		double J;
		double D;// prop disc area
		double shaft_moment_of_inertia_;
		double Tp; // time constant for 

	};
	struct PistonEngineBasicIC{
		double n;
		double prop_position_;
	};
	class PistonPropEngineBasic :
		public Subsystem
	{
	private:
		PistonEngineBasicParam param_;
	public:
		PistonPropEngineBasic();
		~PistonPropEngineBasic();
	};
}
