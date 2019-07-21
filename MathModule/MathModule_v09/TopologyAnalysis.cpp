#include "stdafx.h"
#include "TopologyAnalysis.h"


bool TopologyAnalysis::isnumberinrecord(int index)
{
	bool flag = false;
	// get number of 
	vector<int>::iterator it = find(record.begin(), record.end(), index);
	if (it != record.end())
	{
		flag = true;
	}	
	return flag;
}

void TopologyAnalysis::RemoveDuplicateLoops()
{
}

void TopologyAnalysis::DFS(int level, int index_now)
{
	
	if (level > num_of_nodes)
	{
		return;
	}
	for (int j = 0; j < num_of_nodes; j++)
	{
		if (connectivity_matrix(j,index_now) > 0)
		{
			if (isnumberinrecord(j))
			{				
				num_of_loops++;		
				loopindex.resize(record.size() + 1);
				for (int i = 0; i < record.size(); i++)
				{
					loopindex(i) = record[i];
				}
				loopindex(record.size()) = j;
				loopindexlist.push_back(loopindex);
			}
			else {
				record.push_back(j);
				DFS(level +1, j);
				record.pop_back();
			}

		}
	}

}

void TopologyAnalysis::DisplayClosedLoops()
{
	if (num_of_loops > 0)
	{
		cout << "Total number of closed loops is " << num_of_loops << endl;
		cout << "Node indexes in the closed loops ara" << endl;
		for (int i = 0; i < num_of_loops; i++)
		{
			cout << "Loop # " << i << " : " << endl;
			cout << loopindexlist[i] << endl;
		}
	}
	else
	{
		cout << "No close loops found " << endl;
	}
	

}

void TopologyAnalysis::InitRecord(int index_now)
{
	record.push_back(index_now);
}

TopologyAnalysis::TopologyAnalysis(const MatrixXi& connectivity_matrix_temp)
{
	connectivity_matrix = connectivity_matrix_temp;
	num_of_nodes = connectivity_matrix.cols();
	num_of_loops = 0;
}


TopologyAnalysis::~TopologyAnalysis()
{

}
