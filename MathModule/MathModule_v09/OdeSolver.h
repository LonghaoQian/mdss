/*
���ƣ�OdeSolver
�汾��v0.9 ���԰�
���ߣ�Ǳ���
*/

#pragma once
#include <math.h> 
#include <vector>
#include <Eigen\Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

////////////////////////���״̬�Ľṹ��///////////////////////
//��������Ϣ
struct simulation_step_info{
	VectorXd state;               //ϵͳ״̬
	VectorXd state_derivative;    //ϵͳ״̬��ʱ��ĵ���
	double t;                     //��ǰ����ʱ��
	double step;                  //���沽��
	double eposilon;              //�䲽���в�
};
//ϵͳ��Ϣ
struct simulation_solver_info{
	char solver_name;           //���������
	VectorXd initial_state;     //ϵͳ��ʼ״̬
	int number_of_states;       //ϵͳ��״̬����
	double initial_time;
	double step_max;			//�㷨����󲽳�
	double step_min;			//�㷨����С����	
	MatrixXd butchertableau;    //butchertableau��������
	int num_k;                  //������������е�Ki������
};
//�������ʱ״̬
struct simulation_solver_temp{
	MatrixXd rungekutta_K;//����������������״̬����������ʱk1..kn������ÿһ��Ϊkn��[k1,k2,..,kn]
	MatrixXd update_temp; //��ʱ�ĸ��������ڱ䲽��������������������
};
///////////////////////�궨�����//////////////////////////////
#define INF -1
typedef char solvername;
////////////////////////

///��ⷽ��������
#define DORMANDPRINCE   0x10
#define RUNGKUTTA45     0x11
#define GEARSMETHOD115  0x12
#define LINEARMULTISTEP 0x13

class OdeSolver
{
protected:
//////////////////////////ϵͳ����///////////////////////////
	simulation_step_info step_info;    //ϵͳ��Ϣ
	simulation_solver_info solver_info;//����������Ϣ
	simulation_solver_temp solver_temp;//��ʱ����
////////////////////////////////��ⷽ��///////////////////////////////////
	void InitializeButchertableau(simulation_solver_info& solverinfo,int row,int col);    //��ʼ��butchertableau
	void ode_RungeKuttaFamily(const MatrixXd& butchertableau);			//���������Ĳ���Ki
	VectorXd ode_Increment(const RowVectorXd& butchertableau);           //�ڼ����Ki֮�����butchertableau�������ֵ
public:
////////////////////////////////����״̬///////////////////////////////////
	
	void ode45_DormandPrince(void);//dormand-prince
	void ode45_RungeKutta(void);//runge-kutta
	Matrix3d GetVectorCross(const Vector3d& v);
	OdeSolver(void);
	~OdeSolver(void);//free memory	
   /*
   �����ĳ�΢�ַ��̽ӿڣ��������ж���󸲸�����
   ���̵���ʽΪ��y_dot=f(t,y)��y_dot�ں����в���Ϊ��ʽ����������ڳ�Ա�������������ں������н���������Ա����
   ��������������ٶȵļ���ע�����������
   1.��Ҫ�ں�������������ߴ������飬������룬������Ϊ��̬��������private��Ա������������������������ڴ�����ִֻ��һ�Ρ�
   2.������������� m_temp_solverstatederivative_rungekutta��Ա������
   3.m_temp_solverstate_rungekutta ��Ա����������Initialization������ȷ����С��ſ���ʹ�á�
   4. =����������飬��ĸ��ƣ������ٰ��ิ�Ƹ���һ���࣬��ָ�롣
   */
	virtual VectorXd OrdinaryDifferentialEquation(double& t,//ʱ��
	const VectorXd& state)=0;//ϵͳ״̬
/*������ƺ�����
	���ͷ�����̣�
	<��ʼ��>
	   |
	<״̬����>
		|
 <��λ/���³�ʼ��>
	*/
	
	//����������������ĳ�ʼ��
	void Initialization(solvername name,
		int numuberofstates,
		double initaltime , 
		const VectorXd& initalstate,
		double step);
	//�����䲽��������ĳ�ʼ��
	void Initialization(solvername name,
		int numuberofstates,
		double initaltime, 
		const VectorXd& initalstate,
		double eposilon,
		double minstep,
		double maxstep,
	double initialstep);

	void OneStepUpdate(solvername name);
	//void Reset(void);
	//vector<simulationdata> RunSimulation(double SimulationTime);
	//int RunSimulation(double SimulationTime,double pace);
};

