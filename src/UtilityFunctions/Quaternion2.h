#pragma once
#include <math.h>
#include <Eigen\Dense>
using namespace Eigen;
using namespace std;
class Quaternion2
{
public:
	Vector4d m_lamda_; //四元数向量
	Matrix3d Rbf;//对应方向余弦阵
	Quaternion2(void);
	~Quaternion2(void);
	void RotationMatrixBF(void);//计算旋转矩阵
	void Normalization(void);//归一化
};

