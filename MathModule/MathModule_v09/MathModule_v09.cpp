// MathModule_v09.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "SimController.h"

using namespace std;
int main()
{
	SolverConfig config1;
	config1.eposilon = 0.000001;
	config1.max_step = 0.01;
	config1.mim_step = 0.002;
	config1.solver_type = DORMANDPRINCE;
	SimController SimInstance1(config1);
	// add a LTI system into the subsystem chain
	MatrixXd A(3, 3), B(3, 1), C(1, 3), D(1, 1);
	A << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;
	B << 2, 2, 3;
	C << 1, 1, 0;
	D << 1;
	LTIParameter LTI1;
	LTI1.A = A;
	LTI1.B = B;
	LTI1.C = C;
	LTI1.D = D;
	LTIInitialCondition LTI1IC;
	LTI1IC.X_0.resize(3);
	LTI1IC.X_0(0) = 0.0;
	LTI1IC.X_0(1) = 0.0;
	LTI1IC.X_0(2) = 0.0;
	SimInstance1.AddSubSystem(LTI1, LTI1IC);
	SimInstance1.AddSubSystem(LTI1, LTI1IC);
	SimInstance1.AddSubSystem(LTI1, LTI1IC);
	SimInstance1.AddSubSystem(LTI1, LTI1IC);

	RigidBodyParameter Rigid1;
	Rigid1.J << 1, 0, 0,
		0, 5, 0,
		0, 0, 9;
	Rigid1.m = 9.3;
	RigidBodyCondition RigidIC1;
	RigidIC1.Euler << 0, 0, 0;
	RigidIC1.Omega_BI << 0.2, 0.3, 0.1;
	RigidIC1.V_I << 0, 0, 0.2;
	RigidIC1.X_I << 0, 3, 1;
	SimInstance1.AddSubSystem(Rigid1, RigidIC1);

	MatrixX2i LTI1connection, LTI2connection, LTI3connection, LTI4connection, Rigid1connection;

	LTI1connection.resize(1, 2);

	LTI1connection(0, 0) = 1;
	LTI1connection(0, 1) = 0;

	LTI2connection.resize(1, 2);

	LTI2connection(0, 0) = 3;
	LTI2connection(0, 1) = 0;

	LTI3connection.resize(1, 2);

	LTI3connection(0, 0) = 3;
	LTI3connection(0, 1) = 0;

	LTI4connection.resize(1, 2);
	LTI4connection(0, 0) = 1;
	LTI4connection(0, 1) = 0;

	Rigid1connection.resize(6, 2);

	Rigid1connection(0, 0) = 3;
	Rigid1connection(0, 1) = 0;

	Rigid1connection(1, 0) = 2;
	Rigid1connection(1, 1) = 0;

	Rigid1connection(2, 0) = 1;
	Rigid1connection(2, 1) = 0;

	Rigid1connection(3, 0) = 3;
	Rigid1connection(3, 1) = 0;

	Rigid1connection(4, 0) = -1;
	Rigid1connection(4, 1) = 0;

	Rigid1connection(5, 0) = -1;
	Rigid1connection(5, 1) = 0;



	SimInstance1.MakeConnection(0, LTI1connection);

	SimInstance1.MakeConnection(1, LTI2connection);

	SimInstance1.MakeConnection(2, LTI3connection);

	SimInstance1.MakeConnection(3, LTI4connection);

	SimInstance1.MakeConnection(4, Rigid1connection);

	SimInstance1.PreRunProcess();
	// print the system info
	getchar();
	return 0;
}

