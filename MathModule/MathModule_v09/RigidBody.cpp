#include "StdAfx.h"
#include "RigidBody.h"


RigidBody::RigidBody(void)
{
	m_I_.setZero();
	m_I_(0,0) = 10;
	m_I_(1,1) = 50;
	m_I_(2,2) = 50;
	m_I_inv = m_I_.inverse();
	m_Md_.setZero();
	m_Omegab_(0) = 12.0;
	m_Omegab_(1) = -3.0;
	m_Omegab_(2) = 3.5;
}


RigidBody::~RigidBody(void)
{
}

VectorXd RigidBody::OrdinaryDifferentialEquation(double& t,const VectorXd& state)
{
	VectorXd state_derivative;
	state_derivative = m_I_inv*(m_Md_- GetVectorCross(state)*m_I_*state);
	return state_derivative;
}
