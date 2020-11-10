# C++ 模块化仿真系统

 [![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/Naereen/StrapDown.js/graphs/commit-activity) 

## 正在开发中，目前已经实现多模块拓扑分析，规划和计算功能。

> 1. 基于子系统的仿真程序，利用C++编程实现simulink的基本模块化仿真功能。飞机由各子系统构成，子系统定义在XML文件或者MATLAB  .mat 文件中。程序读取文件，自动解析系统给子系统的串并联结构，实现数值求解微分方程。
> 2. 目前求解器构架已经完成，并且通过多次测试，与Simulink 的计算结果做到完全吻合。
> 3. 正在加入测试更多模块，计划实现simulink aerosapce blockset 的全部模块功能。
> 4. 计划加入 Gaussian Regression 的实现模块，为以后利用机器学习进行QTG系统辨识做基础。
> 5. 求解器自带数据记录功能，同时也编写了MATLAB读取与数据画图脚本。

## 下一个时间点：2020 10月，目标：实现 XML文件载入和仿真预处理优化算法

> 1. 实现利用XML定义模型文件，然后自动读取文件生成仿真模型的功能。
> 2. 实现在预处理阶段进行模型优化的功能。

## 文件目录如下：
|文件目录    |内容                        |
|----------------|-------------------------------|
|src            |`包含所有源代码`            |
|verification       |`包含求解器计算输出和利用Simulink计算的仿真结果的对比验证代码和记录数据读取MATLAB脚本`            |

## 代码编译：

以下步骤包含工程编译步骤 ：(VS2017）
> 第一步：克隆工程到文件夹，解决方案为：solver_test.sln。

> 第二步：下载Eigen库，解压缩后将文件夹重命名为eigen3，放在解决方案文件夹中（ SimulationSolver/）。

> 第三步：选择 release + x64 构型，然后编译运行。

## 模型运行简介：
以下步骤包含运行仿真器的基本设置
> 第一步：定义仿真器基本参数和设定simulationcontrol::SolverConfig，声明仿真器主控类 simulationcontrol::SimController。
示例如下：

	simulationcontrol::SolverConfig config1; // 仿真器参数
	config1.eposilon = 0.00001; // 变步长误差（变步长正在调试）
	config1.adaptive_step = false; // 变步长标识 （变步长正在调试）
	config1.frame_step = 0.02; // 帧时间
	config1.mim_step = 0.005; // 帧内最小时间
	config1.start_time = 0.0; //仿真起始时间
	config1.solver_type = RungeKuttaFamily::RUNGKUTTA45; // 数值方法
	config1.loggingconfig.filename = "datalog.txt"; // 数据记录文件名
	config1.loggingconfig.uselogging = true; // 开启数据记录标识
	config1.loglevel = simulationcontrol::LOGLEVEL_ERROR; // 显示信息
	simulationcontrol::SimController SimInstance1(config1); // 声明仿真系统实例

> 第二步：定义子系统。利用 AddSubSystem(param, initial_condition) 方法添加一个子系统
示例如下：

	linearsystem::IntegratorParameter N2_dynamics; // 定义子系统的参数
	N2_dynamics.num_of_channels = 1;
	linearsystem::IntegratorInitialCondition N2_initialcondition;
	N2_initialcondition.X_0.resize(1);
	N2_initialcondition.X_0(0) = N2_initial;
	// 定义子系统
	unsigned int N2rotordynamics = SimInstance1.AddSubSystem(N2_dynamics, N2_initialcondition); 

> 第三步：连接子系统。使用EditConnectionMatrix(A, a, B, b) 进行连接。将A子系统的第a 个输入连接到B子系统的第b个输出上。然后使用方法FlushMakeConnection()载入连接矩阵。

示例如下：
	
	//将N2_dynamics_sum_1子系统的第0个输入连接到外部输入端 （simulationcontrol::external =-1）
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 0, simulationcontrol::external, 0);
	//将N2_dynamics_sum_1子系统的第1个输入连接到N2rotordynamics子系统的第0个输出端上
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 1, N2rotordynamics, 0);
	//在所有连接完成后，向求解器载入连接矩阵
	SimInstance1.FlushMakeConnection();


> 第四步：预处理。用PreRunProcess()方法对模型进行预处理。预处理函数会分析子系统连接的拓扑结构，检测是否存在代数环，以及确定子系统数值计算顺序。预处理标识符flag为真时，说明预处理成功，系统可以运行。

示例如下：

	bool flag = SimInstance1.PreRunProcess();

> 第五步：在预处理成功，预处理标识符为真时，运行ReshapeExternalInputVector(VectroXd) 方法来确定外部输入向量的大小。Run_Update() 方法运行单步运算。

示例如下：

		VectorXd extern_input; // 声明外部输入向量
		SimInstance1.ReshapeExternalInputVector(extern_input); // 根据预处理结果确定外部输入向量的大小
		SimInstance1.Run_Update(extern_input);// 单步运算

## 数据记录与读取：
>1. 使用类SimController类中的DefineDataLogging(systemID, systemOutputID, "tag_name")方法定义记录的数据。在PreRunProcess()方法运行前使用如下的代码定义记录的数据：

示例如下：

	SimInstance1.DefineDataLogging(N2rotordynamics, 0, "N2_sim");

>2. 在 solver_verification 文件夹中，loadloggeddata.m 为读取数据记录文件的MATLAB函数并将文件数据保存成.mat 格式文件。
>3. 在 solver_verifcation 文件夹中，drawlogfromtag('file_name','tag_name')。
>4. 在solver_verifcation 文件夹中， logfunction_test.m 是一个使用数据读取的示例。

##  算法流程简介（正在更新中）：
查看文档：[模块化仿真系统算法原理](https://www.overleaf.com/project/5ba93e00926a7c3a170dd9bf)

## 目前支持模块列表：
|Aerodynamics      |功能                        |
|----------------|-------------------------------|
|Aeroangle             |`根据空速计算气动角度 （完成）`            |
|Aeroforcement1        |`根据飞行状态和气动角计算气动力和力矩（完成）`            |

|Dynamics      |功能                        |
|----------------|-------------------------------|
|Rigidbody            |`常值质量刚体动力学（完成）`            |
|VairableMassRigidbody            |`变质量量刚体动力学 （计划中）`            |
|OffCenteredRigidbody            |`参考点不在质心的刚体动力学（计划中）`            |

|Geographic      |功能                        |
|----------------|-------------------------------|
|Standard Atomshpere            |`标准大气模型（完成）`            |
|Gravity Model           |`重力模型 （完成）`            |

|Linear System     |功能                        |
|----------------|-------------------------------|
|StateSpace             |`线性系统的状态空间实现（完成）`            |
|Transferfunction        |`线性系统的传递函数实现（完成）`            |
|Integrator        |`积分器（完成）`|
|PID        |` PID 控制器（完成）`|

|Discontinuous System     |功能                        |
|----------------|-------------------------------|
|Saturation           |`限幅器（完成）`            |
|Switch        |`信号切换器（完成）`            |

|Math Blocks      |功能                        |
|----------------|-------------------------------|
|Sum             |`将多个信号做加减法后输出（完成）`            |
|Constant        |`一个常数输出（完成）`            |
|Product        |`将多个信号做乘除法后输出（完成）`|
|CrossProduct        |`将2个3维信号做叉乘后输出（完成）`|
|SpecialFunction       |`特殊函数模块（完成）`|
|TrigonometryFunction        |`三角函数模块（完成）`|
|Lookup1D        |`1维插值（完成）`|
|Lookup2D        |`2维插值（完成）`|

|Machine Learning     |功能                        |
|----------------|-------------------------------|
|Gaussian Regression    |`实现训练好的 Gaussian Regression 网络（计划中）`            |

|Propulsion     |功能                        |
|----------------|-------------------------------|
|Prolleper   |`根据飞行速度计算螺旋桨推力和所需功率（计划中）`            |
|CFM56Aux   |`CFM56 N1，FF,EGT 模型（已完成）`            |
|CFM56ThrustModel  |`CFM56 推力模型（已完成）`            |

|Signal Generator      |功能                        |
|----------------|-------------------------------|
|Step            |`阶跃信号（完成）`            |
|Ramp        |`斜坡信号（完成）`            |
|Periodicwave       |`周期信号：正弦，方波，三角波（完成）`|

|External Model      |功能                        |
|----------------|-------------------------------|
|External Function         |`接受外部定义的复杂函数（计划中）`            |
|External System         |`接受外部定义的复杂系统（计划中）`            |
