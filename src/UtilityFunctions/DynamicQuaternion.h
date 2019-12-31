#pragma once
#include "quaternion2.h"
class DynamicQuaternion :
	public Quaternion2
{
private:
	Matrix<double, 3, 4>  m_R;//��ӦR����
	Matrix<double, 3, 4>  m_L;//��ӦL����
    void UpdateRMatrix(void);
	void UpdateLMatrix(void);
	void Derivative(void);
public:
	DynamicQuaternion(void);
	~DynamicQuaternion(void);
	Vector3d m_omegafb;//��ϵ����ڶ�ϵ�Ľ��ٶ��ڶ�ϵ�е�����
	Vector4d m_lamda_dot;//��Ԫ���ĵ���
	void UpdateQuaternion(void);//������Ԫ����ֵ����һ���������ͷ�����������任����
};

