// MathModule_v09.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "SimController.h"
#include "util_Recorder.h"

using namespace std;
int main()
{
	SolverConfig config1;
	config1.eposilon = 0.0001;
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
	LTIParameter LTI1;

	A << -0.2, 0.1, -1,
		0, -1, 0,
		0.1, 0, -1;
	B << 2, 2, 3;
	
	C << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	D << 0,0,0;
	
	LTI1.A = A;
	LTI1.B = B;
	LTI1.C = C;
	LTI1.D = D; 

	LTIInitialCondition LTI1IC;
	LTI1IC.X_0.resize(3);
	LTI1IC.X_0(0) = 0;
	LTI1IC.X_0(1) = 0;
	LTI1IC.X_0(2) = 0;

	

	SimInstance1.AddSubSystem(LTI1, LTI1IC); 


	MatrixX2i LTI1connection;

	LTI1connection.resize(1, 2);
	LTI1connection(0, 0) = -1;
	LTI1connection(0, 1) = 0;

	SimInstance1.MakeConnection(0, LTI1connection);




	SimInstance1.PreRunProcess();
	// print the system info
	VectorXd extern_input(1);
	extern_input(0) = 1;
	for (int i = 0; i <500; i++)
	{
		SimInstance1.Run_Update(extern_input);
		Data_input[0] = SimInstance1.Run_GetSystemTime();
		for (int j = 0; j < 3; j++)
		{
			Data_input[j+1] = SimInstance1.Run_GetSubsystemOuput(0)(j);
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

