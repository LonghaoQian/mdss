// MathModule_v09.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <Eigen\Dense>
#include "DynamicQuaternion.h"
#include "RigidBody.h"
using Eigen::MatrixXd;
using namespace std;

// 四元数和欧拉角之间的关系

// 四元数的导数和角速度之间的关系


int _tmain(int argc, _TCHAR* argv[])
{
	

	//Matrix<double,2,2> m_r;

	//m_r.Zero();
	//std:cout<<m_r<<endl;


	DynamicQuaternion m_q_1;
	
	m_q_1.m_lamda_(0)=1;
	m_q_1.m_lamda_(1)=-2;
	m_q_1.m_lamda_(2)=0.7;
	m_q_1.m_lamda_(3)=10;
	m_q_1.UpdateQuaternion();
	std::cout<<m_q_1.Rbf<<std::endl;
	std::cout<<m_q_1.m_lamda_<<std::endl;
	std::cout<<m_q_1.m_lamda_dot<<std::endl;

	VectorXd ini(3);
	ini(0)=5;
	ini(1) = 2.3;
	ini(2) = -1.4;

	RigidBody m_body1;

	m_body1.Initialization(DORMANDPRINCE,3,0.0,ini,0.000001,0.0001,0.1,0.02);
	m_body1.OneStepUpdate(DORMANDPRINCE);
	std::cout<<m_body1.m_systemstate_<<std::endl;
	getchar();
	return 0;
}

