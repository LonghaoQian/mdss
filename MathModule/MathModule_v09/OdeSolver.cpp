#include "StdAfx.h"
#include "OdeSolver.h"

OdeSolver::OdeSolver(void)
{

}


OdeSolver::~OdeSolver(void)
{
	
}

void OdeSolver::InitializeButchertableau(simulation_solver_info& solverinfo,int row,int col)
{
	solver_info.butchertableau.setZero(row,col);
	//Dormand Prince 方法参数，除以浮点型的时候要注意加小数点.0(需要记录）
	double t21 = 1.0/5.0;
	double t22 = 1.0/5.0;
	double t31 = 3.0/10.0;
	double t32 = 3.0/40.0;
	double t33 = 9.0/40.0;
	double t41 = 4.0/5.0;
	double t42 = 44.0/45.0;
	double t43 = -56.0/15.0;
	double t44 = 32.0/9.0;
	double t51 = 8.0/9.0;
	double t52 = 19372.0/6561.0;
	double t53 = -25360.0/2187.0;
	double t54 = 64448.0/6561.0;
	double t55 = -212.0/729.0;
	double t61 = 1.0;
	double t62 = 9017.0/3168.0;
	double t63 = -355.0/33.0;
	double t64 = 46732.0/5247.0;
	double t65 = 49.0/176.0;
	double t66 = -5103.0/18656.0;
	double t71 = 1.0;
	double t72 = 35.0/384.0;
	double t73 = 0.0;
	double t74 = 500.0/1113.0;
	double t75 = 125.0/192.0;
	double t76 = -2187.0/6784.0;
	double t77 = 11.0/84.0;

	double t82 = 35.0/384.0;
	double t83 = 0.0;
	double t84 = 500.0/1113.0;
	double t85 = 125.0/192.0;
	double t86 = -2187.0/6784.0;
	double t87 = 11.0/84.0;
	double t88 = 0.0;
	double t92 = 5179.0/57600.0;
	double t93 = 0.0;
	double t94 = 7571.0/16695.0;
	double t95 = 393.0/640.0;
	double t96 = -92097.0/339200.0;
	double t97 = 187.0/2100.0;
	double t98 = 1.0/40.0;

	solver_info.butchertableau(1,0)=t21;
	solver_info.butchertableau(1,1)=t22;
	
	solver_info.butchertableau(2,0)=t31;
	solver_info.butchertableau(2,1)=t32;
	solver_info.butchertableau(2,2)=t33;

	solver_info.butchertableau(3,0)=t41;
	solver_info.butchertableau(3,1)=t42;
	solver_info.butchertableau(3,2)=t43;
	solver_info.butchertableau(3,3)=t44;

	solver_info.butchertableau(4,0)=t51;
	solver_info.butchertableau(4,1)=t52;
	solver_info.butchertableau(4,2)=t53;
	solver_info.butchertableau(4,3)=t54;
	solver_info.butchertableau(4,4)=t55;

	solver_info.butchertableau(5,0)=t61;
	solver_info.butchertableau(5,1)=t62;
	solver_info.butchertableau(5,2)=t63;
	solver_info.butchertableau(5,3)=t64;
	solver_info.butchertableau(5,4)=t65;
	solver_info.butchertableau(5,5)=t66;

	solver_info.butchertableau(6,0)=t71;
	solver_info.butchertableau(6,1)=t72;
	solver_info.butchertableau(6,2)=t73;
	solver_info.butchertableau(6,3)=t74;
	solver_info.butchertableau(6,4)=t75;
	solver_info.butchertableau(6,5)=t76;
	solver_info.butchertableau(6,6)=t77;


	solver_info.butchertableau(7,1)=t82;
	solver_info.butchertableau(7,2)=t83;
	solver_info.butchertableau(7,3)=t84;
	solver_info.butchertableau(7,4)=t85;
	solver_info.butchertableau(7,5)=t86;
	solver_info.butchertableau(7,6)=t87;
	solver_info.butchertableau(7,7)=t88;

	solver_info.butchertableau(8,1)=t92;
	solver_info.butchertableau(8,2)=t93;
	solver_info.butchertableau(8,3)=t94;
	solver_info.butchertableau(8,4)=t95;
	solver_info.butchertableau(8,5)=t96;
	solver_info.butchertableau(8,6)=t97;
	solver_info.butchertableau(8,7)=t98;

	cout<<solver_info.butchertableau<<endl;
	//
	//m_butchertableau_rungekutta
}

void OdeSolver::Initialization(solvername name,
		int numuberofstates,
		double initaltime , 
		const VectorXd& initalstate,
		double step)
{

}

void OdeSolver::Initialization(solvername name,
		int numuberofstates,
		double initaltime , 
		const VectorXd& initalstate,
		double eposilon,
		double minstep,
		double maxstep,
		double initialstep)
{
	solver_info.number_of_states = numuberofstates;  //系统状态个数
	solver_info.initial_time     = initaltime;       //系统初始时间
	solver_info.initial_state    = initalstate;      //系统初始状态
	solver_info.step_max         = maxstep;          //算法的最大步长
	solver_info.step_min         = minstep;			 //算法的最小步长

	int Butchertableau_row = 0;
	int Butchertableau_col = 0;

	switch(name){
	case DORMANDPRINCE:
		solver_info.num_k = 7;
		solver_info.solver_name = DORMANDPRINCE;
		Butchertableau_row = 9;
		Butchertableau_col = 8;
		break;
	case RUNGKUTTA45:
		solver_info.num_k = 4;
		solver_info.solver_name = RUNGKUTTA45;
		break;
	default://默认dormand prince
		solver_info.num_k = 7;
		solver_info.solver_name = DORMANDPRINCE;
		break;
	}
	solver_temp.rungekutta_K.setConstant(solver_info.number_of_states,solver_info.num_k,0);//注意使用Dynamic Size的方法
	InitializeButchertableau(solver_info,Butchertableau_row,Butchertableau_col);
}

void OdeSolver::ode45_DormandPrince(void)
{
//	//变步长Dormand-Prince 方法求解微分方程
//	//计算得出Ki的值
//	ode_RungeKuttaFamily(solver_info.butchertableau);
//	step_info.t += step_info.step;
//
//	//计算残差
//	VectorXd Y;
//	VectorXd Z;
//	ode_Increment(solver_info.butchertableau(8));
//	m_temp_error = m_systemstate_;
//	cout<<m_systemstate_<<endl;
//	ode_Increment(m_butchertableau_dormandprince.row(7));//输出结果
//	cout<<m_systemstate_<<endl;
//	m_temp_error -=m_systemstate_;
//	cout<<m_temp_error.norm()<<endl;
//	// 调整步长
//	m_step *= pow((0.5*m_eposilon*m_step/m_temp_error.norm()),0.2);
//	cout<<m_step<<endl;
//	if(m_step>m_step_max)
//	{
//		m_step = m_step_max;
//	}else
//	{
//		if(m_step<m_step_min)
//		{
//			m_step = m_step_min;
//		}
//	}
//	
}

void OdeSolver::ode45_RungeKutta(void)
{


}
/////////////////////////////////////////////////数值方法/////////////////////////////////////////////////////////
/*
龙格库塔方法的一般表达式为：

计算ki的函数为ode_RungeKuttaFamily
计算数值增量的函数为ode_Increment
*/

void OdeSolver::ode_RungeKuttaFamily(const MatrixXd& butchertableau)
{
	VectorXd m_temp_solverstate_rungekutta;
	for(int i =0;i<solver_info.num_k;i++)//龙格库塔类的求解方法
	{
		m_temp_solverstate_rungekutta= step_info.state;//重置为系统状态
		for(int j=0;j<i;j++)
		{
			m_temp_solverstate_rungekutta+=butchertableau(i,j+1)*solver_temp.rungekutta_K.col(j);//变化状态
		}
		//计算Ki
		double t_temp = step_info.t+butchertableau(i,0)*step_info.step;
		solver_temp.rungekutta_K.col(i)= OrdinaryDifferentialEquation(t_temp, m_temp_solverstate_rungekutta);//将Ki存入内存中
	}
}

VectorXd OdeSolver::ode_Increment(const RowVectorXd& butchertableau)
{
	VectorXd state_update;
	state_update = step_info.state;
	for(int i = 0;i<solver_info.num_k;i++)
	{
		state_update +=  step_info.step*solver_temp.rungekutta_K.col(i)*butchertableau(i+1);
	}
	return state_update;
}
///////////////////////////////////////////////---------------////////////////////////////////////////////////////
Matrix3d OdeSolver::GetVectorCross(const Vector3d& v)
{
	Matrix3d m;
	m.setZero();

	m(0,1)= -v(2);
	m(0,2) = v(1);
	m(1,0) = v(2);
	m(1,2) = -v(0);
	m(2,0) = -v(1);
	m(2,1) = v(0);

	return m;
}

void OdeSolver::OneStepUpdate(solvername name)
{
	switch(name){
	case DORMANDPRINCE:
		ode45_DormandPrince();
		break;
	case RUNGKUTTA45:
		ode45_RungeKutta();
		break;
	default:
		break;
	}
}

//vector<simulationdata> OdeSolver::RunSimulation(double SimulationTime)
//{
//	//速度比较慢，用来分析运动曲线。
//	vector<simulationdata> m_solution;//初始化动态矩阵，用来储存计算变量
//	simulationdata onestepsolution;//零时储存变量
//	//输入初始化的结果
//	onestepsolution.m_eposilon=0;
//	onestepsolution.m_step = m_step;
//	onestepsolution.m_systemstate_derivative = m_systemstate_derivative;
//	onestepsolution.m_t = 0;
//	onestepsolution.m_systemstate_ = m_systemstate_;
//
//	bool exitflag = true; //退出标示
//	while(exitflag)
//	{
//		if(SimulationTime-onestepsolution.m_t<m_step)
//		{
//			m_step = SimulationTime-m_t_;
//			exitflag = false;}
//		ode45_DormandPrince();
//		//将结果存入零时变量
//		onestepsolution.m_eposilon=m_eposilon;
//		onestepsolution.m_step = m_step;
//		onestepsolution.m_systemstate_derivative = m_systemstate_derivative;
//		onestepsolution.m_t = m_t_;
//		onestepsolution.m_systemstate_ = m_systemstate_;
//			//推入结果中
//		m_solution.push_back(onestepsolution);
//	}
//	return m_solution;
//}