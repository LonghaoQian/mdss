#include "StdAfx.h"
#include "DynamicQuaternion.h"


DynamicQuaternion::DynamicQuaternion(void)
{
	//��ʼ����Ԫ��
	 m_lamda_(0) = 1;
	 m_lamda_(1) = 0;
	 m_lamda_(2) = 0;
	 m_lamda_(3) = 0;

	 m_omegafb(0) =0;
	 m_omegafb(1) =0;
	 m_omegafb(2) =0;

	 UpdateQuaternion();
}


DynamicQuaternion::~DynamicQuaternion(void)
{
}

/*
��Ԫ����R����
R=def=[-lamda lamdaX+lamda0*E3]=
[-lamda_1 lamda_0 -lamda_3 lamda_2;
-lamda_2 lamda_3 lamda_0 -lamda_1;
-lamda_3 -lamda_2 lamda_1 lamda_0];
*/
void DynamicQuaternion::UpdateRMatrix(void)
{
	//����R�����ֵ
	for(int i=0;i<3;i++)
	{
			 m_R(i,1)=-m_lamda_(i+1);
			 m_R(i,i+1)=m_lamda_(0);
	};
	 m_R(0,2)=-m_lamda_(3);
	 m_R(0,3)= m_lamda_(2);
	 m_R(1,1)= m_lamda_(3);
     m_R(1,3)=-m_lamda_(1);
     m_R(2,1)=-m_lamda_(2);
	 m_R(2,2)= m_lamda_(1);
}

/*
��Ԫ����L����
R=def=[-lamda -lamdaX+lamda0*E3]=
[-lamda_1 lamda_0 lamda_3 -lamda_2;
-lamda_2 -lamda_3 lamda_0 lamda_1;
-lamda_3 lamda_2 -lamda_1 lamda_0];
*/

void DynamicQuaternion::UpdateLMatrix(void)
{
	//����L�����ֵ
		for(int i=0;i<3;i++)
	{
			 m_L(i,1)=-m_lamda_(i+1);
			 m_L(i,i+1)=m_lamda_(0);
	};
	 m_L(0,2)=  m_lamda_(3);
	 m_L(0,3)= -m_lamda_(2);
	 m_L(1,1)= -m_lamda_(3);
     m_L(1,3)=  m_lamda_(1);
     m_L(2,1)=  m_lamda_(2);
	 m_L(2,2)= -m_lamda_(1);
}

void  DynamicQuaternion::Derivative(void)
{
	UpdateLMatrix();
	m_lamda_dot=0.5*m_L.transpose()*m_omegafb;
}

void DynamicQuaternion::UpdateQuaternion(void)
{
	Normalization();   //��һ��
	Derivative();      //���µ���
	RotationMatrixBF();//���·���������
}