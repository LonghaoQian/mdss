#pragma once
#include "odesolver.h"
class RigidBody :
	public OdeSolver
{
public:
	Matrix3d m_I_;//������
	Matrix3d m_I_inv;//���������
	Vector3d m_Fb_;//��������������ϵ�ϵ�����
	Vector3d m_Md_;//����������������ϵ�ϵ�����
	Vector3d m_Omegab_;//����Ľ��ٶ�
	Matrix3d m_OmegaCross_;//���ٶȵ�ʸ����
	VectorXd  OrdinaryDifferentialEquation(double& t,//ʱ��
	const VectorXd& state);
	RigidBody(void);
	~RigidBody(void);
};

