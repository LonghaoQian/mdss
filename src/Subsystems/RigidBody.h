/*
_________________________________
Author: Longhao Qian
Data:   2020 09 09

dynamic blocks contains the following blocks:

1. rigid body block
2. rigid kinematics block
3. rigid dynamics block
TO DO:
4. offcentered rigid dynamics block
________________________________
*/
#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"
namespace dynamics {

	enum RigidbodyOutput {
		RIGIDBODY_OUTPUT_VIx = 0,
		RIGIDBODY_OUTPUT_VIy,
		RIGIDBODY_OUTPUT_VIz,
		RIGIDBODY_OUTPUT_omega_BIx,
		RIGIDBODY_OUTPUT_omega_BIy,
		RIGIDBODY_OUTPUT_omega_BIz,
		RIGIDBODY_OUTPUT_XIx,
		RIGIDBODY_OUTPUT_XIy,
		RIGIDBODY_OUTPUT_XIz,
		RIGIDBODY_OUTPUT_R_IB00,
		RIGIDBODY_OUTPUT_R_IB10,
		RIGIDBODY_OUTPUT_R_IB20,
		RIGIDBODY_OUTPUT_R_IB01,
		RIGIDBODY_OUTPUT_R_IB11,
		RIGIDBODY_OUTPUT_R_IB21,
		RIGIDBODY_OUTPUT_R_IB02,
		RIGIDBODY_OUTPUT_R_IB12,
		RIGIDBODY_OUTPUT_R_IB22,
		RIGIDBODY_OUTPUT_VBx,
		RIGIDBODY_OUTPUT_VBy,
		RIGIDBODY_OUTPUT_VBz
	};


	struct RigidBodyParameter {
		Matrix3d J;
		double m;
	};
	struct RigidBodyCondition {
		Vector3d X_I;
		Vector3d V_I;
		Vector3d Omega_BI;
		Vector3d Euler;
	};
	/*

	output(0, 1, 2) = V_I
	output(3, 4, 5) = omega_BI
	output(6, 7, 8) = X_I
	output(9, 10, 11) = R_IB(:, 1)
	output(12, 13, 14) = R_IB(:, 2)
	output(15, 16, 17) = R_IB(:, 3)
	output(18, 19, 20) = V_B

	*/

	class RigidBody :
		public Subsystem
	{
	private:
		Matrix3d J;// moment of inertia
		Matrix3d J_inv; // the inverse of moment of inertia
		double m;// mass
		Matrix3d R_IB;
		Vector4d q;// quaternion
	public:
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		RigidBody(const RigidBodyParameter& parameter, const RigidBodyCondition& IC);
		~RigidBody();
	};

	enum RigidBodyKinematicsInput {
		KINEMATICS_INPUT_AIx = 0,
		KINEMATICS_INPUT_AIy,
		KINEMATICS_INPUT_AIz,
		KINEMATICS_INPUT_OMEGA_DOTx,
		KINEMATICS_INPUT_OMEGA_DOTy,
		KINEMATICS_INPUT_OMEGA_DOTz,
	};

	enum RigidBodyKinematicsOutput {
		KINEMATICS_OUTPUT_VBx = 0,
		KINEMATICS_OUTPUT_VBy,
		KINEMATICS_OUTPUT_VBz,
		KINEMATICS_OUTPUT_VIx,
		KINEMATICS_OUTPUT_VIy,
		KINEMATICS_OUTPUT_VIz,
		KINEMATICS_OUTPUT_XIx,
		KINEMATICS_OUTPUT_XIy,
		KINEMATICS_OUTPUT_XIz,
		KINEMATICS_OUTPUT_OmegaBIx,
		KINEMATICS_OUTPUT_OmegaBIy,
		KINEMATICS_OUTPUT_OmegaBIz,
		KINEMATICS_OUTPUT_R_IB00,
		KINEMATICS_OUTPUT_R_IB10,
		KINEMATICS_OUTPUT_R_IB20,
		KINEMATICS_OUTPUT_R_IB01,
		KINEMATICS_OUTPUT_R_IB11,
		KINEMATICS_OUTPUT_R_IB21,
		KINEMATICS_OUTPUT_R_IB02,
		KINEMATICS_OUTPUT_R_IB12,
		KINEMATICS_OUTPUT_R_IB22,
		KINEMATICS_OUTPUT_EulerRoll,
		KINEMATICS_OUTPUT_EulerPitch,
		KINEMATICS_OUTPUT_EulerYaw,
	};

	enum RigidBodyKinematicsState {
		KINEMATICS_STATE_VIx = 0,
		KINEMATICS_STATE_VIy,
		KINEMATICS_STATE_VIz,
		KINEMATICS_STATE_OmegaBIx,
		KINEMATICS_STATE_OmegaBIy,
		KINEMATICS_STATE_OmegaBIz,
		KINEMATICS_STATE_XIx,
		KINEMATICS_STATE_XIy,
		KINEMATICS_STATE_XIz,
		KINEMATICS_STATE_q0,
		KINEMATICS_STATE_q1,
		KINEMATICS_STATE_q2,
		KINEMATICS_STATE_q3,
		KINEMATICS_STATE_q4,
	};


	struct RigidBodyKinematicsInitialCondition {
		Eigen::Vector3d XI0;
		Eigen::Vector3d VI0;
		Eigen::Vector3d Euler0;
		Eigen::Vector3d Omega0;
	};

	class RigidBodyKinematics :
		public Subsystem {
	private:
		RigidBodyKinematicsInitialCondition InitialCondition;
		Vector4d quaternion;
		Matrix3d R_IB;
	public:
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		RigidBodyKinematics(const RigidBodyKinematicsInitialCondition& IC);
		~RigidBodyKinematics();
	};

	struct RigidBodyDynamicsParamter {
		Matrix3d J;// moment of inertia
		double m;// mass
	};


	enum RigidBodyDynamicsInput {
		DYNAMICS_INPUT_FIx = 0,
		DYNAMICS_INPUT_FIy,
		DYNAMICS_INPUT_FIz,
		DYNAMICS_INPUT_TBx,
		DYNAMICS_INPUT_TBy,
		DYNAMICS_INPUT_TBz,
		DYNAMICS_INPUT_OmegaBIx,
		DYNAMICS_INPUT_OmegaBIy,
		DYNAMICS_INPUT_OmegaBIz,
	};

	enum RigidBodyDynamicsOutput {
		DYNAMICS_OUTPUT_AIx = 0,
		DYNAMICS_OUTPUT_AIy,
		DYNAMICS_OUTPUT_AIz,
		DYNAMICS_OUTPUT_OMEGA_DOTx,
		DYNAMICS_OUTPUT_OMEGA_DOTy,
		DYNAMICS_OUTPUT_OMEGA_DOTz,
	};

	class RigidBodyDynamics :
		public Subsystem {
	private:
		Matrix3d J;// moment of inertia
		Matrix3d J_inv; // the inverse of moment of inertia
		double m;// mass
	public:
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		RigidBodyDynamics(const RigidBodyDynamicsParamter& parameter);
		~RigidBodyDynamics();
	};
	// TO DO: gyroscopic forces for rotating objects
}
