/*
名称：OdeSolver
版本：v0.9 测试版
作者：潜龙昊
*/

#pragma once
#include <math.h> 
#include <vector>
#include <Eigen\Dense>
#include <iostream>
using namespace Eigen;
using namespace std;

////////////////////////求解状态的结构体///////////////////////
//单步的信息
struct simulation_step_info{
	VectorXd state;               //系统状态
	VectorXd state_derivative;    //系统状态对时间的导数
	double t;                     //当前步的时间
	double step;                  //仿真步长
	double eposilon;              //变步长残差
};
//系统信息
struct simulation_solver_info{
	char solver_name;           //求解器名称
	VectorXd initial_state;     //系统初始状态
	int number_of_states;       //系统的状态数量
	double initial_time;
	double step_max;			//算法的最大步长
	double step_min;			//算法的最小步长	
	MatrixXd butchertableau;    //butchertableau参数矩阵
	int num_k;                  //龙格库塔方法中的Ki的数量
};
//求解器临时状态
struct simulation_solver_temp{
	MatrixXd rungekutta_K;//龙格库塔类求解器的状态，用来放临时k1..kn变量，每一列为kn，[k1,k2,..,kn]
	MatrixXd update_temp; //临时的更新量，在变步长方法中用来调整步长
};
///////////////////////宏定义参数//////////////////////////////
#define INF -1
typedef char solvername;
////////////////////////

///求解方法的名称
#define DORMANDPRINCE   0x10
#define RUNGKUTTA45     0x11
#define GEARSMETHOD115  0x12
#define LINEARMULTISTEP 0x13

class OdeSolver
{
protected:
//////////////////////////系统参数///////////////////////////
	simulation_step_info step_info;    //系统信息
	simulation_solver_info solver_info;//求解器相关信息
	simulation_solver_temp solver_temp;//临时变量
////////////////////////////////求解方法///////////////////////////////////
	void InitializeButchertableau(simulation_solver_info& solverinfo,int row,int col);    //初始化butchertableau
	void ode_RungeKuttaFamily(const MatrixXd& butchertableau);			//龙格库塔族的参量Ki
	VectorXd ode_Increment(const RowVectorXd& butchertableau);           //在计算出Ki之后根据butchertableau计算更新值
public:
////////////////////////////////仿真状态///////////////////////////////////
	
	void ode45_DormandPrince(void);//dormand-prince
	void ode45_RungeKutta(void);//runge-kutta
	Matrix3d GetVectorCross(const Vector3d& v);
	OdeSolver(void);
	~OdeSolver(void);//free memory	
   /*
   待求解的常微分方程接口，在子类中定义后覆盖它；
   方程的形式为：y_dot=f(t,y)，y_dot在函数中不作为显式输出，而是在成员变量中声明，在函数体中将结果输入成员变量
   ！！！提高运行速度的几点注意事项！！！：
   1.不要在函数中声明类或者大型数组，如果必须，可以设为静态，或者在private成员变量中声明，尽量做到大块内存声明只执行一次。
   2.将导数结果放在 m_temp_solverstatederivative_rungekutta成员变量中
   3.m_temp_solverstate_rungekutta 成员变量必须在Initialization函数中确定大小后才可以使用。
   4. =避免大型数组，类的复制，尽量少把类复制给另一个类，用指针。
   */
	virtual VectorXd OrdinaryDifferentialEquation(double& t,//时间
	const VectorXd& state)=0;//系统状态
/*仿真控制函数：
	典型仿真过程：
	<初始化>
	   |
	<状态更新>
		|
 <复位/重新初始化>
	*/
	
	//单步定步长求解器的初始化
	void Initialization(solvername name,
		int numuberofstates,
		double initaltime , 
		const VectorXd& initalstate,
		double step);
	//单步变步长求解器的初始化
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

