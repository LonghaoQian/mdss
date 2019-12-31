#pragma once
#include <Eigen\Dense>
#include <iostream>
#include <vector>
using namespace Eigen;
using namespace std;
class TopologyAnalysis
{
private:
	MatrixXi connectivity_matrix;// 
	int num_of_nodes;
	vector<int> record;
	vector<int> index_temp;
	vector<VectorXi> loopindexlist;
	VectorXi loopindex;
	vector<bool>  IsIndexScanned;
	int temp_duplicate_index;
	int num_of_loops;
	bool isnumberinrecord(int index);
	void DFS(int level, int index_now);
	void DisplayClosedLoops();
	void InitRecord(int index_now);
public:
	int RunSimulationTopologyAnalysis();
	VectorXi GetLoopIndex(int loop_index);
	TopologyAnalysis(const MatrixXi& connectivity_matrix_temp);
	~TopologyAnalysis();
};

