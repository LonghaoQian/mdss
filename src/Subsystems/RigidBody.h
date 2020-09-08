#pragma once
#include "Subsystem.h"
#include "UtilityFunctions.h"

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

enum RigidbodyState{
	v_Ix = 0,
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

