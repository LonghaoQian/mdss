#include "stdafx.h"
#include "TopologyAnalysis.h"


bool TopologyAnalysis::isnumberinrecord(int index)
{
	bool flag = false;
	// get number of 
	vector<int>::iterator it = find(record.begin(), record.end(), index);
	if (it != record.end())
	{
		temp_duplicate_index = distance(record.begin(), it);
		flag = true;
	}	
	return flag;
}

void TopologyAnalysis::DFS(int level, int index_now)
{
	
    IsIndexScanned[index_now] = true;// updated is scanned list
	if (level > num_of_nodes)
	{
		return;
	}
	for (int j = 0; j < num_of_nodes; j++)
	{
		if (connectivity_matrix(j,index_now) > 0)
		{
			if (isnumberinrecord(j))// true if a loop has been found
			{				
				loopindex.resize(record.size()- temp_duplicate_index);
				index_temp.clear();
				for (int i = temp_duplicate_index; i < record.size(); i++)
				{
					index_temp.push_back(record[i]);
				}
				// sort the index temp
				sort(index_temp.begin(), index_temp.end());
				for (int i = 0; i < index_temp.size(); i++)
				{
					loopindex(i) = index_temp[i];
				}
				bool isduplicated = false;
				VectorXi temp;
				if (num_of_loops > 0)// determine whether this loop has been found when num_of_loop is greater than 0
				{
					for (int i = 0; i < loopindexlist.size(); i++)
					{
						if (loopindexlist[i].size() == loopindex.size())// if size is the same then check elements
						{
							temp = loopindexlist[i] - loopindex;
							if ((temp.minCoeff()==0)&&(temp.maxCoeff()==0))
							{
								isduplicated = true;
							}
						}
					}
				}
				if (isduplicated == false)
				{
					loopindexlist.push_back(loopindex);	
					num_of_loops = loopindexlist.size();
				}
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
		cout << "No closed loops found " << endl;
	}
	

}

int TopologyAnalysis::RunSimulationTopologyAnalysis()
{
	int start_index = 0;
	bool scanning_continue = true;
	int unscanned_index;
	while (scanning_continue)
	{
		vector<bool>::iterator it = find(IsIndexScanned.begin(), IsIndexScanned.end(), false);
		if (it != IsIndexScanned.end())
		{
			// there are unscanned nodes
			start_index = distance(IsIndexScanned.begin(), it);
			InitRecord(start_index);
			DFS(0, start_index);
		}
		else {
			scanning_continue = false;
		// no unscanned nodes
		}	
	}
	DisplayClosedLoops();
	return num_of_loops;
}

VectorXi TopologyAnalysis::GetLoopIndex(int loop_index)
{
	VectorXi index_temp;
	if (num_of_loops > 0)
	{
		index_temp.resize(loopindexlist[loop_index].size());
		index_temp =loopindexlist[loop_index];
	}
	else {
		index_temp.resize(1);
		index_temp(0) = -1;
	}
	return index_temp;
}

void TopologyAnalysis::InitRecord(int index_now)
{
	record.clear();
	record.push_back(index_now);
}

TopologyAnalysis::TopologyAnalysis(const MatrixXi& connectivity_matrix_temp)
{
	connectivity_matrix = connectivity_matrix_temp;
	num_of_nodes = connectivity_matrix.cols();
	num_of_loops = 0;
	for (int i = 0; i < num_of_nodes; i++)
	{
		IsIndexScanned.push_back(false);
	}
}


TopologyAnalysis::~TopologyAnalysis()
{

}
