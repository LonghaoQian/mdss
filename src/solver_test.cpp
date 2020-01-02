// solver_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "SimController.h"
#include "util_Recorder.h"
using std::iostream;
int main()
{
	SolverConfig config1;
	config1.eposilon = 0.00001;
	config1.adaptive_step = true;
	config1.frame_step = 0.02;
	config1.mim_step = 0.005;
	config1.start_time = 0.0;
	config1.solver_type = DORMANDPRINCE;
	SimController SimInstance1(config1);

	util_Recorder Recorder1("LTIsimulation.txt", 4);
	double Data_input[4];

	// add a LTI system into the subsystem chain
	MatrixXd A(3, 3), B(3, 1), C(3, 3), D(3, 1);
	LTIParameter LTI0, LTI1;

	A << -0.2, 0.1, -1,
		0, -1, 0,
		0.1, 0, -1;
	B << 2, 2, 3;

	C << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	D << 0, 0, 0;

	LTI0.A = A;
	LTI0.B = B;
	LTI0.C = C;
	LTI0.D = D;

	MatrixXd A1(2, 2), B1(2, 1), C1(2, 2), D1(2, 1);
	A1 << 0, -0.1,
		1, 0;
	B1 << 0, 1;
	C1 << 1, 0,
		0, 1;
	D1 << 0, 0;
	LTI1.A = A1;
	LTI1.B = B1;
	LTI1.C = C1;
	LTI1.D = D1;

	LTIInitialCondition LTI0IC, LTI1IC;
	LTI0IC.X_0.resize(3);
	LTI0IC.X_0(0) = 0;
	LTI0IC.X_0(1) = 0;
	LTI0IC.X_0(2) = 0;

	LTI1IC.X_0.resize(2);
	LTI1IC.X_0(0) = 0;
	LTI1IC.X_0(1) = 1;


	Gainparameter Gain1, Gain2;
	Gain1.K.resize(1, 1);
	Gain1.K(0, 0) = -2;
	Gain1.num_of_inputs = 1;
	Gain1.type = GAIN_SCALER;

	Gain2.K.resize(1, 2);
	Gain2.K(0, 0) = -1;
	Gain2.K(0, 1) = -0.2;
	Gain2.type = GAIN_MATRIX;


	SimInstance1.AddSubSystem(LTI0, LTI0IC);
	SimInstance1.AddSubSystem(Gain1);
	SimInstance1.AddSubSystem(LTI1, LTI1IC);
	SimInstance1.AddSubSystem(Gain2);
	MatrixX2i LTI0connection, Gain1connection, LTI1connection, Gain2connection;

	LTI0connection.resize(1, 2);
	LTI0connection(0, 0) = 3;
	LTI0connection(0, 1) = 0;

	SimInstance1.MakeConnection(0, LTI0connection);

	Gain1connection.resize(1, 2);
	Gain1connection(0, 0) = 0;
	Gain1connection(0, 1) = 0;

	SimInstance1.MakeConnection(1, Gain1connection);

	LTI1connection.resize(1, 2);
	LTI1connection(0, 0) = 1;
	LTI1connection(0, 1) = 0;

	SimInstance1.MakeConnection(2, LTI1connection);

	Gain2connection.resize(2, 2);
	Gain2connection(0, 0) = 2;
	Gain2connection(0, 1) = 0;
	Gain2connection(1, 0) = 2;
	Gain2connection(1, 1) = 1;


	SimInstance1.MakeConnection(3, Gain2connection);

	SimInstance1.PreRunProcess();
	// print the system info
	VectorXd extern_input(1);
	extern_input(0) = 1;
	for (int i = 0; i < 1000; i++)
	{
		SimInstance1.Run_Update(extern_input);
		Data_input[0] = SimInstance1.Run_GetSystemTime();
		for (int j = 0; j < 3; j++)
		{
			Data_input[j + 1] = SimInstance1.Run_GetSubsystemOuput(0)(j);
		}
		Recorder1.Record(Data_input);
		//cout << "t= " << SimInstance1.Run_GetSystemTime() << endl;
		//cout << " Output = " << SimInstance1.Run_GetSubsystemOuput(0) << endl;
	}
	cout << "t= " << SimInstance1.Run_GetSystemTime() << endl;
	cout << " Output = " << SimInstance1.Run_GetSubsystemOuput(0) << endl;
	getchar();
	return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
