#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace groundcontact {
	/*--------------  simple gear contact force based on point contact model ---------------------*/
	struct SimpleGearNormalForceParameter{
		Vector3d GearPosition;  // natural location of the landing gear 
		Vector3d GearDirection; // the direction of the landing gear compression (if the gear has a complex suspension, this is the equivalent direction of compression)
		double kCompress{0.0};    // the stiffness parameter of impact phase
		double kRebound{0.0};     // the stiffness parameter of the rebound phase
		double dCompress{0.0};    // the damping parameter of the impact phase
		double dRebound{0.0};     // the damping parameter of the rebound phase
		double MaxHeight{ 1.0 };   // the maximum height where the gear block starts calculation NED frame
		double MinNz{ 0.0 };      // gear block  will stop calculation when the third component of the z axis of the aircraft body fixed frame is lower than this value
	};

	enum GearNormalForceInput {
		GEARNORMAL_INPUT_SWITCH = 0,// the switch controlling the calculation of the gear model
		GEARNORMAL_INPUT_PIx,       // the aircraft position in an local ground frame (NED)
		GEARNORMAL_INPUT_PIy,
		GEARNORMAL_INPUT_PIz,
		GEARNORMAL_INPUT_VIx,       // the inertial speed of the aircraft in the local ground frame (NED)
		GEARNORMAL_INPUT_VIy,
		GEARNORMAL_INPUT_VIz,
		GEARNORMAL_INPUT_OMEGAbx,   // the angular velocity of the aircraft in the body-fixed frame
		GEARNORMAL_INPUT_OMEGAby,
		GEARNORMAL_INPUT_OMEGAbz,
		GEARNORMAL_INPUT_R_IB00,    // the rotation matrix between the body-fixed frame the inertial frame
		GEARNORMAL_INPUT_R_IB10,
		GEARNORMAL_INPUT_R_IB20,
		GEARNORMAL_INPUT_R_IB01,
		GEARNORMAL_INPUT_R_IB11,
		GEARNORMAL_INPUT_R_IB21,
		GEARNORMAL_INPUT_R_IB02,
		GEARNORMAL_INPUT_R_IB12,
		GEARNORMAL_INPUT_R_IB22,
		GEARNORMAL_INPUT_H,        // the ground height at the projected contact point (NED)
		GEARNORMAL_INPUT_Hdot,     // the rate change of the ground height at the projected contact point (NED)
		GEARNORMAL_INPUT_NGx,     // the ground normal vector in local inertial frame
		GEARNORMAL_INPUT_NGy,
		GEARNORMAL_INPUT_NGz,
	};

	enum GearNormalForceOutput {
		GEARNORMAL_OUTPUT_COMPRESSION = 0, // the compression distance of the landing gear;
		GEARNORMAL_OUTPUT_COMPRESSIONRATE, // the compression rate of the landing gear;
		GEARNORMAL_OUTPUT_Nbx,               // the normal force of the landing gear relative to the ground in body-fixed frame
		GEARNORMAL_OUTPUT_Nby,
		GEARNORMAL_OUTPUT_Nbz,
		GEARNORMAL_OUTPUT_F,               // the force along the landing gear suspension relative to the aircraft
	};

	class SimpleGearNormalForce:
		public Subsystem
	{
	private:
		SimpleGearNormalForceParameter param_;
		Vector3d re;
		Vector3d ne;
		Matrix3d R_IB;
		Matrix3d R_IB_dot;
		Vector3d re_dot;
		Vector3d ne_dot;
	public:
		SimpleGearNormalForce(const SimpleGearNormalForceParameter& param);
		void DifferentialEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input,
			VectorXd& derivative);
		void OutputEquation(const double& t,
			const VectorXd& state,
			const VectorXd& input, VectorXd& output);
		void IncrementState();
		void DisplayParameters();
		void DisplayInitialCondition();
		~SimpleGearNormalForce();
	};
	/*--------------  simple gear friction force based on LuGre model ---------------------*/
	struct LuGreFrictionParameter {
		double rollingfriction{ 0.0 };
	};
	/*--------------  TO DO: ground height model ---------------------*/
}