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
	MatrixXd B1(3, 2), C1(2, 3), D1(2, 2), B2(3, 3);
	RigidBodyParameter Rigid1, Rigid2, Rigid3;

	Rigid1.J << 1, 0, 0,
		0, 5, 0,
		0, 0, 9;
	Rigid1.m = 9.3;

	Rigid2.J << 1, 0, 0,
		0, 5, 0,
		0, 0, 9;
	Rigid2.m = 9.3;

	LTIParameter LTI1, LTI2, LTI3, LTI4, LTI5;

	A << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;
	B << 2, 2, 3;
	
	C << 1, 1, 0;
	D << 1;
	
	C1<< 2, 2, 3,
		2, 2, 3;

	B1 << 2, 2, 3,
		1, 1, 0;

	D1 << 1, 1,
		1, 1;


	LTI1.A = A;
	LTI1.B = B;
	LTI1.C = C;
	LTI1.D = D; 

	LTI2.A = A;
	LTI2.B = B1;
	LTI2.C = C1;
	LTI2.D = D;

	LTI3.A = A;
	LTI3.B = B;
	LTI3.C = C;
	LTI3.D = D;

	LTI4.A = A;
	LTI4.B = A;
	LTI4.C = C;
	LTI4.D = D;

	LTI5.A = A;
	LTI5.B = B;
	LTI5.C = C;
	LTI5.D = D;
	
	LTIInitialCondition LTI1IC, LTI12C, LTI13C, LTI14C, LTI15C;
	LTI1IC.X_0.resize(3);
	LTI1IC.X_0(0) = 0.0;
	LTI1IC.X_0(1) = 0.0;
	LTI1IC.X_0(2) = 0.0;

	RigidBodyCondition RigidIC1, RigidIC2, RigidIC3;
	RigidIC1.Euler << 0, 0, 0;
	RigidIC1.Omega_BI << 0.2, 0.3, 0.1;
	RigidIC1.V_I << 0, 0, 0.2;
	RigidIC1.X_I << 0, 3, 1;

	RigidIC2 = RigidIC1;
	RigidIC3 = RigidIC1;

	SimInstance1.AddSubSystem(LTI1, LTI1IC); 
	SimInstance1.AddSubSystem(LTI2, LTI1IC);
	SimInstance1.AddSubSystem(LTI3, LTI1IC);
	SimInstance1.AddSubSystem(Rigid1, RigidIC1);
	SimInstance1.AddSubSystem(LTI4, LTI1IC);
	SimInstance1.AddSubSystem(Rigid2, RigidIC2);
	SimInstance1.AddSubSystem(LTI5, LTI15C);
	SimInstance1.AddSubSystem(Rigid3, RigidIC3);

	MatrixX2i LTI1connection, LTI2connection, LTI3connection, LTI4connection, LTI5connection, Rigid1connection, Rigid2connection, Rigid3connection;

	LTI1connection.resize(1, 2);
	LTI1connection(0, 0) = -1;
	LTI1connection(0, 1) = 0;

	LTI2connection.resize(2, 2);
	LTI2connection(0, 0) = 0;
	LTI2connection(0, 1) = 0;
	LTI2connection(1, 0) = 7;
	LTI2connection(1, 1) = 0;

	LTI3connection.resize(1, 2);
	LTI3connection(0, 0) = 1;
	LTI3connection(0, 1) = 0;

	LTI4connection.resize(3, 2);
	LTI4connection(0, 0) = 2;
	LTI4connection(0, 1) = 0;
	LTI4connection(1, 0) = 5;
	LTI4connection(1, 1) = 0;
	LTI4connection(2, 0) = 7;
	LTI4connection(2, 1) = 0;

	LTI5connection.resize(1, 2);
	LTI5connection(0, 0) = 7;
	LTI5connection(0, 1) = 0;



	Rigid1connection.resize(6, 2);
	Rigid1connection(0, 0) = 1;
	Rigid1connection(0, 1) = 0;
	Rigid1connection(1, 0) = 4;
	Rigid1connection(1, 1) = 0;
	Rigid1connection(2, 0) = 6;
	Rigid1connection(2, 1) = 0;
	Rigid1connection(3, 0) = -1;
	Rigid1connection(3, 1) = 0;
	Rigid1connection(4, 0) = -1;
	Rigid1connection(4, 1) = 0;
	Rigid1connection(5, 0) = -1;
	Rigid1connection(5, 1) = 0;

	Rigid2connection.resize(6, 2);
	Rigid2connection(0, 0) = -1;
	Rigid2connection(0, 1) = 0;
	Rigid2connection(1, 0) = -1;
	Rigid2connection(1, 1) = 0;
	Rigid2connection(2, 0) = -1;
	Rigid2connection(2, 1) = 0;
	Rigid2connection(3, 0) = -1;
	Rigid2connection(3, 1) = 0;
	Rigid2connection(4, 0) = -1;
	Rigid2connection(4, 1) = 0;
	Rigid2connection(5, 0) = -1;
	Rigid2connection(5, 1) = 0;

	Rigid3connection.resize(6, 2);
	Rigid3connection(0, 0) = -1;
	Rigid3connection(0, 1) = 0;
	Rigid3connection(1, 0) = -1;
	Rigid3connection(1, 1) = 0;
	Rigid3connection(2, 0) = -1;
	Rigid3connection(2, 1) = 0;
	Rigid3connection(3, 0) = -1;
	Rigid3connection(3, 1) = 0;
	Rigid3connection(4, 0) = -1;
	Rigid3connection(4, 1) = 0;
	Rigid3connection(5, 0) = -1;
	Rigid3connection(5, 1) = 0;


	SimInstance1.MakeConnection(0, LTI1connection);

	SimInstance1.MakeConnection(1, LTI2connection);

	SimInstance1.MakeConnection(2, LTI3connection);

	SimInstance1.MakeConnection(3, Rigid1connection);

	SimInstance1.MakeConnection(4, LTI4connection);

	SimInstance1.MakeConnection(5, Rigid2connection);

	SimInstance1.MakeConnection(6, LTI5connection);

	SimInstance1.MakeConnection(7, Rigid3connection);


	SimInstance1.PreRunProcess();
	// print the system info
	getchar();
	return 0;
}

