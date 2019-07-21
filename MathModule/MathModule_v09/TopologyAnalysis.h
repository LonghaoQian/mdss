#pragma once
#include <Eigen\Dense>
#include <iostream>
#include <vector>
using namespace Eigen;
using namespace std;
class TopologyAnalysis
{
private:
	MatrixXi ToplogyList;
	MatrixXi connectivity_matrix;
	int num_of_nodes;
	vector<int> record;
	vector<VectorXi> loopindexlist;
	VectorXi loopindex;
	int num_of_loops;
	bool isnumberinrecord(int index);
	void RemoveDuplicateLoops();
public:
	void DFS(int level, int index_now);
	void DisplayClosedLoops();
	void InitRecord(int index_now);
	TopologyAnalysis(const MatrixXi& connectivity_matrix_temp);
	~TopologyAnalysis();
};

