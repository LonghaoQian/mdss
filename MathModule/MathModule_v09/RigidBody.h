#pragma once
#include "odesolver.h"
class RigidBody :
	public OdeSolver
{
public:
	Matrix3d m_I_;//惯量阵
	Matrix3d m_I_inv;//惯量阵的逆
	Vector3d m_Fb_;//合外力在体坐标系上的坐标
	Vector3d m_Md_;//合外力矩在体坐标系上的坐标
	Vector3d m_Omegab_;//刚体的角速度
	Matrix3d m_OmegaCross_;//角速度的矢量阵
	VectorXd  OrdinaryDifferentialEquation(double& t,//时间
	const VectorXd& state);
	RigidBody(void);
	~RigidBody(void);
};

