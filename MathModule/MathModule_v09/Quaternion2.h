#pragma once
#include <math.h>
#include <Eigen\Dense>
using namespace Eigen;
using namespace std;
class Quaternion2
{
public:
	Vector4d m_lamda_; //��Ԫ������
	Matrix3d Rbf;//��Ӧ����������
	Quaternion2(void);
	~Quaternion2(void);
	void RotationMatrixBF(void);//������ת����
	void Normalization(void);//��һ��
};

