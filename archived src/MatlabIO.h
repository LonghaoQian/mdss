#pragma once
#include <Eigen\Dense>
#include <iostream>
#include <vector>
#include <string>
#include "mat.h" // matlab API
using std::string;
using std::vector;
class MatlabIO
{
private:
	MATFile *pmat_;
	const char *file_name_;
public:
	MatlabIO();
	bool ReadFromMatFile(Eigen::MatrixXd& data_,
						 const char*file, 
						 const char *variable_name);
	bool SaveToMatFile(const Eigen::MatrixXd& variable_, 
					   const char* file_name, 
					   const char* variable_name);
	~MatlabIO();
};

