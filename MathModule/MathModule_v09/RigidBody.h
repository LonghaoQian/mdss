#pragma once
#include "Subsystem.h"
#include <iostream>
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
	VectorXd GetState();
	RigidBody();
	RigidBody(const RigidBodyParameter& parameter, const RigidBodyCondition& IC);
	~RigidBody();
};

