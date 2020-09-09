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

	enum RigidbodyState {
		RIGIDBODY_OUTPUT_vIx = 0,
		v_Iy,
		v_Iz,
		omega_BIx,
		omega_BIy,
		omega_BIz,
		x_Ix,
		x_Iy,
		x_Iz,
		R_IB00,
		R_IB10,
		R_IB20,
		R_IB01,
		R_IB11,
		R_IB21,
		R_IB02,
		R_IB12,
		R_IB22,
		v_Bx,
		v_By,
		v_Bz
	};

	enum RigidbodyOutput {

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
	public:
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		RigidBody();
		RigidBody(const RigidBodyParameter& parameter, const RigidBodyCondition& IC);
		~RigidBody();
	};

	enum KinematicsInput {
		KINEMATICS_INPUT_AIx = 0,
		KINEMATICS_INPUT_AIy,
		KINEMATICS_INPUT_AIz,
		KINEMATICS_INPUT_OMEGA_DOTx,
		KINEMATICS_INPUT_OMEGA_DOTy,
		KINEMATICS_INPUT_OMEGA_DOTz,
	};

	enum KinematicsOutput {
		KINEMATICS_OUTPUT_VBx,
		KINEMATICS_OUTPUT_VBy,
		KINEMATICS_OUTPUT_VBz,
		KINEMATICS_OUTPUT_VIx,
		KINEMATICS_OUTPUT_VIy,
		KINEMATICS_OUTPUT_VIz,
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
	};

	class RigidBodyKinematics :
		public Subsystem {
	private:
		Matrix3d R_IB;
	public:
		void DifferentialEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& derivative);
		void OutputEquation(const double& t, const VectorXd& state, const VectorXd& input, VectorXd& output);
		void DisplayParameters();
		void DisplayInitialCondition();
		void IncrementState();
		RigidBodyKinematics();
		RigidBodyKinematics(const RigidBodyParameter& parameter, const RigidBodyCondition& IC);
		~RigidBodyKinematics();
	};



}
