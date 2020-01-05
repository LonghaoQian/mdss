#include "pch.h"
#include "MatlabIO.h"


MatlabIO::MatlabIO()
{
}


MatlabIO::~MatlabIO()
{
}


bool MatlabIO::ReadFromMatFile(Eigen::MatrixXd& data_, 
							   const char *file,
							   const char* variable_name) {

	file_name_ = file;// save file name
	// open MAT-file
	pmat_ = matOpen(file_name_, "r");

	if (pmat_ == NULL) {
		std::cout << "Can not open: " << file_name_ << std::endl;
		return false;
	}
	int D1_X = 0;
	int D2_X = 0;
	// extract the specified variable
	mxArray *arr = matGetVariable(pmat_, variable_name);

	vector<double> v; // a temp storage

	if (arr != NULL && mxIsDouble(arr) && !mxIsEmpty(arr)) {
		// copy data
		mwSize num_of_dimensions = mxGetNumberOfDimensions(arr); // determine number of dimensions
		mwSize num = mxGetNumberOfElements(arr);
		//std::cout << "Number of total data is: " << num << std::endl;
		double *pr = mxGetPr(arr);
		if (pr != NULL) {
			v.reserve(num); //is faster than resize :-)
			v.assign(pr, pr + num);
			if (num_of_dimensions == 2) {
				D1_X = mxGetDimensions(arr)[0];
				D2_X = mxGetDimensions(arr)[1];
				//std::cout << " 1st dimension #: " << D1_X << std::endl;
				//std::cout << " 2st dimension #: " << D2_X << std::endl;
				data_.resize(D1_X, D2_X);
				data_.setZero();
				//std::cout << "data_: " << data_(0,0)<< std::endl;
				for (int i = 0; i < D2_X; i++) {
					for (int j = 0; j < D1_X; j++) {
						//std::cout << " i #: " << i << std::endl;
						//std::cout << " j #: " << j << std::endl;
						//std::cout << " total: #: " << i * D2_X + j << std::endl;
						data_(j, i) = v[i*D1_X+j]; // put 1D array to matrix
						//std::cout << " data is : " <<  data_(i, j) << std::endl;
					}
				}
			} else {
				//if over 2, treat the value as a 1 D array
				data_.resize(num, 1);
				data_.setZero();
				for (int i = 0; i < num; i++) {
					data_(i, 1) = v[i];
				}
			}

		}
		else {
			std::cout << "Can not load pointer to the data ! " << std::endl;
			return false;
		}
	}
	else {
		std::cout << "No data or incorrect data type ! " << std::endl;
		return false;
	}

	// cleanup
	std::cout << "Successfully load: " << file_name_ << std::endl;
	mxDestroyArray(arr);
	matClose(pmat_);
	return true;
}

bool  MatlabIO::SaveToMatFile(const Eigen::MatrixXd& variable_, 
							  const char* file_name, 
							  const char* variable_name) {
	// open MAT-file
	pmat_ = matOpen(file_name, "w");

	if (pmat_ == NULL) {
		return false;
	}
	return true;
}