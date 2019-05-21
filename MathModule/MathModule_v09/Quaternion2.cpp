#include "StdAfx.h"
#include "Quaternion2.h"

Quaternion2::Quaternion2(void)
{
	//初始化四元数
	 m_lamda_(0) = 1;
	 m_lamda_(1) = 0;
	 m_lamda_(2) = 0;
	 m_lamda_(3) = 0;
}

Quaternion2::~Quaternion2(void)
{
}

/*将四元数输出为方向余弦阵


输入为准备修改方向余弦阵的引用
*/
void Quaternion2::RotationMatrixBF(void)
{
	Rbf(0,0)=2*(m_lamda_(0)*m_lamda_(0)+m_lamda_(1)*m_lamda_(1))-1;
	Rbf(1,0)=2*(m_lamda_(1)*m_lamda_(2)+m_lamda_(0)*m_lamda_(3));
	Rbf(2,0)=2*(m_lamda_(1)*m_lamda_(3)-m_lamda_(0)*m_lamda_(2));

	Rbf(0,1)=2*(m_lamda_(1)*m_lamda_(2)-m_lamda_(0)*m_lamda_(3));
	Rbf(1,1)=2*(m_lamda_(0)*m_lamda_(0)+m_lamda_(2)*m_lamda_(2))-1;
	Rbf(2,1)=2*(m_lamda_(2)*m_lamda_(3)+m_lamda_(0)*m_lamda_(1));

	Rbf(0,2)=2*(m_lamda_(1)*m_lamda_(3)+m_lamda_(0)*m_lamda_(2));
	Rbf(1,2)=2*(m_lamda_(2)*m_lamda_(3)-m_lamda_(0)*m_lamda_(1));
	Rbf(2,2)=2*(m_lamda_(0)*m_lamda_(0)+m_lamda_(3)*m_lamda_(3))-1;
}

void Quaternion2::Normalization(void)
{
	m_lamda_.normalize();
}
