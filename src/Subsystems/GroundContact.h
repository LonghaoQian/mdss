#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace groundcontact {
	/*--------------  simple gear contact force based on point contact model ---------------------*/
	struct SimpleGearNormalForceParameter{
		Vector3d GearPosition;    // natural location of the landing gear 
		Vector3d GearDirection;   // the direction of the landing gear compression (if the gear has a complex suspension, this is the equivalent direction of compression)
		double kCompress{0.0};    // the stiffness parameter of impact phase
		double dCompress{0.0};    // the damping parameter of the impact phase
		double dRebound{0.0};     // the damping parameter of the rebound phase
		double MaxHeight{ 1.0 };  // the maximum height where the gear block starts calculation NED frame (a positive number )
		double MinNz{ 0.0 };      // gear block  will stop calculation when the third component of the z axis of the aircraft body fixed frame is lower than this value
		bool isSteering{ false };   // is steering enabled
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
		GEARNORMAL_INPUT_NGx,      // the ground normal vector in local inertial frame
		GEARNORMAL_INPUT_NGy,
		GEARNORMAL_INPUT_NGz,
		GEARNORMAL_INPUT_STEERING, // the steering angle (deg)
	};

	enum GearNormalForceOutput {
		GEARNORMAL_OUTPUT_COMPRESSION = 0, // the compression distance of the landing gear;
		GEARNORMAL_OUTPUT_COMPRESSIONRATE, // the compression rate of the landing gear;
		GEARNORMAL_OUTPUT_Nbx,             // the normal force of the landing gear relative to the ground in body-fixed frame
		GEARNORMAL_OUTPUT_Nby,
		GEARNORMAL_OUTPUT_Nbz,
		GEARNORMAL_OUTPUT_NIx,             // the normal force of the landing gear relative to the ground in inertial frame
		GEARNORMAL_OUTPUT_NIy,
		GEARNORMAL_OUTPUT_NIz,
		GEARNORMAL_OUTPUT_Mbx,             // moment created by the normal force in body-fixed frame
		GEARNORMAL_OUTPUT_Mby,
		GEARNORMAL_OUTPUT_Mbz,
		GEARNORMAL_OUTPUT_VGIw,            // the speed of the contact point relative to the ground expressed in the body-fixed frame
		GEARNORMAL_OUTPUT_VGIp,            // the wheel lateral speed 
		GEARNORMAL_OUTPUT_nwIx,            // the wheel longitudinal direction
		GEARNORMAL_OUTPUT_nwIy,
		GEARNORMAL_OUTPUT_nwIz,
		GEARNORMAL_OUTPUT_npIx,            // the wheel lateral direction
		GEARNORMAL_OUTPUT_npIy,
		GEARNORMAL_OUTPUT_npIz,
		GEARNORMAL_OUTPUT_F,               // the force along the landing gear suspension relative to the aircraft
		GEARNORMAL_OUTPUT_CONTACTPOINTx,   // the inertial position of the wheel contact point
		GEARNORMAL_OUTPUT_CONTACTPOINTy,
		GEARNORMAL_OUTPUT_CONTACTPOINTz,
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
		Matrix3d NGG_2;
		Matrix3d Identity3;
		Vector3d re_dot;
		Vector3d ne_dot;
		Matrix3d R_WB;      // the rotation matrix from the wheel frame to the body-fixed frame
		Vector3d nwb;      // the vector that is along the rolling direction of the wheel
		Vector3d npb;      // the vector that is perpendicular to the wheel plane
		Vector3d nwI;      // the 
		Vector3d npI;      // the 
		Vector3d VgI;      // the wheel speed in inertial frame
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
		double RollingFrictionCoefficient{ 0.0 };
		double DynamicFrictionCoefficient{ 0.0 };
		double StaticFrictionCoefficient{ 0.0 };
		double StiffnessSigma0{ 0.0 };
		double DampingSigma1{ 0.0 };
		double SwitchSigmaS{ 0.0 };
		double SwitchSigmaD{ 0.0 };
	};


	enum GearLuGreFrictionInput {
		LUGREFRICTION_INPUT_VwI = 0, // wheel longitudinal speed
		LUGREFRICTION_INPUT_VpI,     // wheel lateral speed
		LUGREFRICTION_INPUT_NIx,     // normal force in inertial frame
		LUGREFRICTION_INPUT_NIy,
		LUGREFRICTION_INPUT_NIz,
		LUGREFRICTION_INPUT_Nwx,     // the wheel longditudinal direction in inertial frame
		LUGREFRICTION_INPUT_Nwy,
		LUGREFRICTION_INPUT_Nwz,
		LUGREFRICTION_INPUT_Npx,     // the wheel lateral direction in inertial frame
		LUGREFRICTION_INPUT_Npy,
		LUGREFRICTION_INPUT_Npz,
		LUGREFRICTION_INPUT_CPx,     // the wheel contact point relative to the body-fixed frame  expressed in inertial frame
		LUGREFRICTION_INPUT_CPy,
		LUGREFRICTION_INPUT_CPz,
		LUGREFRICTION_INPUT_BREAKING,
	};

	enum GearLuGreFrictionOutput {
		LUGREFRICTION_OUTPUT_FIx = 0,// the wheel force expressed in the inertial frame
		LUGREFRICTION_OUTPUT_FIy,
		LUGREFRICTION_OUTPUT_FIz,
		LUGREFRICTION_OUTPUT_MIx,    // the wheel momment to the center of the body-fixed frame expressed in inertial frame
		LUGREFRICTION_OUTPUT_MIy,
		LUGREFRICTION_OUTPUT_MIz,
	};

	enum GearLuGreFrictionState {
		LUGREFRICTION_STATE_ROLL_Z = 0,
		LUGREFRICTION_STATE_SIDE_Z,
	};

	struct GearLuGreFrictionParameter {
		double RollingFrictionCoefficient{ 0.0 };
		double StaticFrictionCoefficient{ 0.0 };
		double DynamicFrictionCoefficient{ 0.0 };
		double StiffnessSigma0{ 0.0 };
		double DampingSigma1{ 0.0 };
		double SwitchSigmaS{ 0.0 };
		double SwitchSigmaD{ 0.0 };
		double vlimit{ 0.0 };
	};

	class GearLuGreFriction :
		public Subsystem
	{
	public:
		GearLuGreFriction(const GearLuGreFrictionParameter& parameter);
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
		~GearLuGreFriction();
	private:
		GearLuGreFrictionParameter param_;
		double RollingStaticFriction{ 0.0 };
		double RollingDynamicFriction{ 0.0 };
		double deltaFriction1{0.0};
		double LugreZdynamics(const double& z,const double& v, const double& sigma_dynamic, const double& sigam_static);
		double TotalRollingFriction;
		double TotalSideFriction;
		double vw;
		double vp;
		Vector3d CPI;
		Vector3d FI;
	};

	/*--------------  TO DO: ground height model ---------------------*/

	/*--------------  TO DO: pure friction model ---------------------*/
}