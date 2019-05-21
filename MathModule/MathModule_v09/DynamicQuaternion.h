#pragma once
#include "quaternion2.h"
class DynamicQuaternion :
	public Quaternion2
{
private:
	Matrix<double, 3, 4>  m_R;//对应R矩阵
	Matrix<double, 3, 4>  m_L;//对应L矩阵
    void UpdateRMatrix(void);
	void UpdateLMatrix(void);
	void Derivative(void);
public:
	DynamicQuaternion(void);
	~DynamicQuaternion(void);
	Vector3d m_omegafb;//动系相对于定系的角速度在动系中的坐标
	Vector4d m_lamda_dot;//四元数的导数
	void UpdateQuaternion(void);//更新四元数的值、归一化、导数和方向余弦坐标变换矩阵
};

