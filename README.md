﻿# C++ 仿真程序


## 正在开发中，目前已经实现基本计算功能


> 1. 基本功能主要是基于结构的仿真程序，在C++实模拟simulink的基本仿真功能。飞机由各子系统构成，子系统定义在XML文件中。程序读取文件，自动解析系统给子系统的串并联结构，实现数值求解微分方程
> 2. 目前基本求解器已经完成，正在经行和simulink仿真结果的对比。在对比几个算例之后，再对程序进行优化。
> 3. 在基本计算功能实现之后，将加入算例仿真演示程序。

## 下一个时间点：2020 12月底，目标：实现C172基本系统的仿真。

> 1. 基本系统包括，气动，发动机和起落架


## 算例演示


>1. 目前没有图形界面，所以程序运行比较抽象，仿真结果由MATLAB画图显示。目前实现的是C++仿真求解计算的核心功能。运行:
`\SimulationSolver\MathModule\x64\Release\MathModule_v09.exe` 
然后运行:
`\SimulationSolver\MathModule\x64\Release\testLTI1.m` 
查看仿真结果的对比。
在这个算例中，C++仿真器和Simulink对于同一个动力学模型进行仿真，输出仿真结果进行对比。 动力学模型可以打开`LTI_test1.slx` 查看。算例对比LTI0系统状态在仿真中的结果，即Y1的3个分量。
根据结果显示，这两个程序的结果一致，可以初步认定C++仿真器的计算功能运行正确。
> 2. 这个仿真求解器功能不仅限于给定的飞机，它是一个可以仿真一般系统的仿真器，和simulink的功能类似。复杂的动力学模型，如飞机，利用程序内置的子系统模块搭建而成。在输入输出模块完成时，用户无需编程即可实现仿真。同时可以任意改变子系统的结构，如添加额外的飞行控制律等。
> 3. 下一步计划实现一个简单的XML文件读取功能，和基本的建模文档读取功能。
> 4. 这个C++仿真求解器是以后对于SDSA以及其他飞行仿真和飞行控制设计项目的基础。