# C++ 模块化仿真系统


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
|solver_verification       |`包含求解器计算输出和利用Simulink计算的仿真结果的对比验证代码和记录数据读取MATLAB脚本`            |

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
	simulationcontrol::SimController SimInstance1(config1); // 声明仿真系统实例

> 第二步：定义子系统。利用 AddSubSystem(param, initial_condition) 方法添加一个子系统
示例如下：

	std::vector< subsystem_handle*> handle_pointer_list; // 定义一个子系统指针的容器

	linearsystem::IntegratorParameter N2_dynamics; // 定义子系统的参数
	N2_dynamics.num_of_channels = 1;
	linearsystem::IntegratorInitialCondition N2_initialcondition;
	N2_initialcondition.X_0.resize(1);
	N2_initialcondition.X_0(0) = N2_initial;
	// 定义子系统
	subsystem_handle N2rotordynamics = SimInstance1.AddSubSystem(N2_dynamics, N2_initialcondition); 
	// 将子系统指针推入容器内
	handle_pointer_list.push_back(&N2rotordynamics);

> 第三步：连接子系统。使用EditConnectionMatrix(A, a, B, b) 进行连接。将A子系统的第a 个输入连接到B子系统的第b个输出上。使用方法MakeConnection()载入连接矩阵。

示例如下：
	
	//将N2_dynamics_sum_1子系统的第0个输入连接到外部输入端 （simulationcontrol::external =-1）
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 0, simulationcontrol::external, 0);
	//将N2_dynamics_sum_1子系统的第1个输入连接到N2rotordynamics子系统的第0个输出端上
	SimInstance1.EditConnectionMatrix(N2_dynamics_sum_1, 1, N2rotordynamics.ID, 0);
	// 得出子系统总数
	int num_of_systems_defined = handle_pointer_list.size();
	// 载入连接矩阵
	for (int i = 0; i < num_of_systems_defined; i++) {
		SimInstance1.MakeConnection(*handle_pointer_list[i]);
	}


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

	SimInstance1.DefineDataLogging(N2rotordynamics.ID, 0, "N2_sim");

>2. 在 solver_verification 文件夹中，loadloggeddata.m 为读取数据记录文件的MATLAB函数并将文件数据保存成.mat 格式文件。
>3. 在 solver_verifcation 文件夹中，drawlogfromtag('file_name','tag_name')。
>4. 在solver_verifcation 文件夹中， logfunction_test.m 是一个使用数据读取的示例。

##  算法流程简介（正在更新中）：
查看文档：[模块化仿真系统算法原理](https://www.overleaf.com/project/5ba93e00926a7c3a170dd9bf)