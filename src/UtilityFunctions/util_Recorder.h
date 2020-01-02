#pragma once
#include <sstream>
#include <fstream>
#include <iostream>
using namespace std;

class util_Recorder
{
private:
	int data_counter;
	const char* filename;
	unsigned int numbe_of_column;  //dimension of the data
	ofstream file;                 //file object that store the
public:
	void ResetRecorder();
	void Record(double* data);
	util_Recorder(const char* _filename, unsigned int _num_of_colums);
	~util_Recorder();
};

