#include "pch.h"
#include "UtilityFunctions.h"
Vector3d mathauxiliary::Veemap(const Matrix3d& cross_matrix) {
	Vector3d vector;
	vector(0) = -cross_matrix(1, 2);
	vector(1) = cross_matrix(0, 2);
	vector(2) = -cross_matrix(0, 1);
	return vector;

}
Matrix3d mathauxiliary::Hatmap(const Vector3d& vector) {
	/*
	r^x = [0 -r3 r2;
	r3 0 -r1;
	-r2 r1 0]
	*/
	Matrix3d cross_matrix;
	cross_matrix(0, 0) = 0.0;
	cross_matrix(0, 1) = -vector(2);
	cross_matrix(0, 2) = vector(1);

	cross_matrix(1, 0) = vector(2);
	cross_matrix(1, 1) = 0.0;
	cross_matrix(1, 2) = -vector(0);

	cross_matrix(2, 0) = -vector(1);
	cross_matrix(2, 1) = vector(0);
	cross_matrix(2, 2) = 0.0;
	return cross_matrix;
}

Vector3d mathauxiliary::GetEulerAngleFromQuaterion(const Vector4d& quaterion) {
	/* Normal means the following https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	*/
	// roll (x-axis rotation)
	Vector3d Euler;
	double sinr_cosp = +2.0 * (quaterion(0) * quaterion(1) + quaterion(2) * quaterion(3));
	double cosr_cosp = +1.0 - 2.0 * (quaterion(1) * quaterion(1) + quaterion(2) * quaterion(2));
	double roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (quaterion(0) *quaterion(2) - quaterion(3) * quaterion(1));
	double pitch;
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (quaterion(0) * quaterion(3) + quaterion(1) * quaterion(2));
	double cosy_cosp = +1.0 - 2.0 * (quaterion(2)* quaterion(2) + quaterion(3) * quaterion(3));
	double yaw = atan2(siny_cosp, cosy_cosp);
	Euler(0) = roll;
	Euler(1) = pitch;
	Euler(2) = yaw;
	return Euler;
}
Vector4d mathauxiliary::GetQuaterionFromRulerAngle(const Vector3d& Euler) {
	Vector4d Quaterion;
	// Abbreviations for the various angular functions
	double cy = cos(Euler(2) * 0.5);
	double sy = sin(Euler(2)* 0.5);
	double cp = cos(Euler(1) * 0.5);
	double sp = sin(Euler(1) * 0.5);
	double cr = cos(Euler(0) * 0.5);
	double sr = sin(Euler(0) * 0.5);

	Quaterion(0) = cy * cp * cr + sy * sp * sr;
	Quaterion(1) = cy * cp * sr - sy * sp * cr;
	Quaterion(2) = sy * cp * sr + cy * sp * cr;
	Quaterion(3) = sy * cp * cr - cy * sp * sr;

	return Quaterion;
}

Matrix<double, 3, 4> mathauxiliary::GetLmatrixFromQuaterion(const Vector4d & quaterion)
{
    Matrix<double, 3, 4> Lmatrix;
	// update the auxiliary matrix
	/*
	L = [-q1 q0 q3 -q2;
	-q2 -q3 q0 q1;
	-q3 q2 -q1 q0]
	R = [-q1 q0 -q3 q2;
	-q2 q3 q0 -q1;
	-q3 -q2 q1 q0]
	R_IB = RL^T
	*/
	Lmatrix(0, 0) = -quaterion(1);
	Lmatrix(1, 0) = -quaterion(2);
	Lmatrix(2, 0) = -quaterion(3);

	Lmatrix(0, 1) = quaterion(0);
	Lmatrix(1, 2) = quaterion(0);
	Lmatrix(2, 3) = quaterion(0);

	Lmatrix(0, 2) = quaterion(3);
	Lmatrix(0, 3) = -quaterion(2);
	Lmatrix(1, 1) = -quaterion(3);
	Lmatrix(1, 3) = quaterion(1);
	Lmatrix(2, 1) = quaterion(2);
	Lmatrix(2, 2) = -quaterion(1);

	return Lmatrix;
}

Matrix<double, 3, 4> mathauxiliary::GetRmatrixFromQuaterion(const Vector4d & quaterion)
{
	Matrix<double, 3, 4> Rmatrix;
	Rmatrix(0, 0) = -quaterion(1);
	Rmatrix(1, 0) = -quaterion(2);
	Rmatrix(2, 0) = -quaterion(3);

	Rmatrix(0, 1) = quaterion(0);
	Rmatrix(1, 2) = quaterion(0);
	Rmatrix(2, 3) = quaterion(0);

	Rmatrix(0, 2) = -quaterion(3);
	Rmatrix(0, 3) = quaterion(2);
	Rmatrix(1, 1) = quaterion(3);
	Rmatrix(1, 3) = -quaterion(1);
	Rmatrix(2, 1) = -quaterion(2);
	Rmatrix(2, 2) = quaterion(1);
	
	return Rmatrix;
}

Matrix3d mathauxiliary::GetR_IBFromQuaterion(const Vector4d & quaterion)
{
	// pose[1].R * pose[1].L.transpose();
	Matrix3d R_IB = GetRmatrixFromQuaterion(quaterion)  *  GetLmatrixFromQuaterion(quaterion).transpose();
	return R_IB;
}

Matrix<double, 9, 1> mathauxiliary::ConvertRotationMatrixToVector(const Matrix<double, 3, 3>& R)
{
   Matrix<double, 9, 1> v;

   v.block<3, 1>(0, 0) = R.block<3, 1>(0, 0);
   v.block<3, 1>(3, 0) = R.block<3, 1>(0, 1);
   v.block<3, 1>(6, 0) = R.block<3, 1>(0, 2);
	return v;
}

Matrix<double, 3, 3> mathauxiliary::ConvertVectorToRotationMatrix(const Matrix<double, 9, 1>& v)
{
	Matrix<double, 3, 3> R;
	R.block<3, 1>(0, 0) = v.block<3, 1>(0, 0);
	R.block<3, 1>(0, 1) = v.block<3, 1>(3, 0);
	R.block<3, 1>(0, 2) = v.block<3, 1>(6, 0);
	return R;
}

Vector2i mathauxiliary::BinarySearchVector(bool isascending, 
										   const VectorXd& p, 
										   const double& target) {
	// 
	Vector2i indexarray;
	// initialize index array
	indexarray << 0,
				  p.size() - 1;
	// Corner cases 
	if (isascending) {
		if (target <= p(0)) {
			indexarray << 0,
				0;
			return indexarray;
		}
		if (target >= p(p.size() - 1)) {
			indexarray << p.size() - 1,
				p.size() - 1;
			return indexarray;
		}
	}
	else {
		if (target >= p(0)) {
			indexarray << 0,
				0;
			return indexarray;
		}
		if (target <= p(p.size() - 1)) {
			indexarray << p.size() - 1,
				p.size() - 1;
			return indexarray;
		}
	}

	// Doing binary search 
	int mid = 0;

	if (isascending) {
		while (indexarray(0) < indexarray(1) - 1) {
			mid = (indexarray(0) + indexarray(1)) / 2;

			if (p(mid) <= target) {
				indexarray(0) = mid;
			}
			else {
				indexarray(1) = mid;
			}
		}
	}
	else {
		while (indexarray(0) < indexarray(1) - 1) {
			mid = (indexarray(0) + indexarray(1)) / 2;

			if (p(mid) >= target) {
				indexarray(0) = mid;
			}
			else {
				indexarray(1) = mid;
			}
		}
	}

	return indexarray;
}

double mathauxiliary::LinearInterpolation1D(const VectorXd& data, 
										    int index1, 
											int index2, 
											const VectorXd& reference,
											const double& target) {
	return data(index1) + (data(index2) - data(index1)) * (target - reference(index1))/(reference(index2) - reference(index1));
}

double mathauxiliary::LinearInterpolation2D(const MatrixXd & data, 
											const Vector2i & index_1d, 
											const Vector2i & index_2d, 
											const VectorXd & reference_1d, 
											const VectorXd & reference_2d,
											const double & target) {
	// bilinear interpolation
	double temp_1, temp_2;
	temp_1 = LinearInterpolation1D(data.col(index_2d(0)), index_1d(0), index_1d(1), reference_1d, target);
	temp_2 = LinearInterpolation1D(data.col(index_2d(1)), index_1d(0), index_1d(1), reference_1d, target);

	VectorXd temp_3;
	temp_3 << temp_1,
			  temp_2;

	return LinearInterpolation1D(temp_3, index_2d(0), index_2d(1), reference_2d, target);
}

mathauxiliary::LookupInterface::LookupInterface(void)
{

}

mathauxiliary::LookupInterface::~LookupInterface(void)
{

}

mathauxiliary::Lookup_1D::Lookup_1D() {

}

mathauxiliary::Lookup_1D::~Lookup_1D(void)
{
	
}

void mathauxiliary::Lookup_1D::GetOutput(double& output, const double& target)
{
	index_sequence_ = BinarySearchVector(true, reference_1d_, target);
	if (!ismulti_) {
		if (index_sequence_(0) == index_sequence_(1)){ 
			if (isextrapolation) {

			}
			else {
				output = table_data_(index_sequence_(0));
			}
		}
		else {
			output = LinearInterpolation1D(table_data_, index_sequence_(0), index_sequence_(1), reference_1d_, target);
		}
	} 
}

void mathauxiliary::Lookup_1D::GetOutput(VectorXd & output, const double & target)
{
	index_sequence_ = BinarySearchVector(true, reference_1d_, target);
	if (ismulti_) {
		if (output.size() != num_of_data_arrays) {
			output.resize(num_of_data_arrays);
		}
		if (index_sequence_(0) == index_sequence_(1)) {
			if (isextrapolation) {
			}
			else {
				for (int i = 0; i < num_of_data_arrays; i++) {
					output(i) = table_data_multi_.col(i)(index_sequence_(0));
				}
			}
		}
		else {
			for (int i = 0; i < num_of_data_arrays; i++) {
				output(i) = LinearInterpolation1D(table_data_multi_.col(i), index_sequence_(0), index_sequence_(1), reference_1d_, target);
			}
		}
	}
}

void mathauxiliary::Lookup_1D::LoadTableData(const VectorXd & reference_1d, 
											 const MatrixXd & _data,
											 bool extrapolation)
{
	isextrapolation = extrapolation;
	num_of_data_arrays = _data.cols();
	num_of_references_ = reference_1d.size();
	reference_1d_ = reference_1d;
	if (num_of_data_arrays == 1)
	{
		table_data_.resize(num_of_references_);
		table_data_ = _data;
	}
	else {
		table_data_multi_.resize(_data.rows(), _data.cols());
		table_data_multi_ = _data;
	}

	Preprocess();// sort the data in ascending order
}

void mathauxiliary::Lookup_1D::Preprocess()
{
	// sort the data in ascending order
	//std::sort(input_1.b, input_1.end());
	if (num_of_data_arrays == 1) {
		ismulti_ = false;
	} else {
		ismulti_ = true;
	}
}

mathauxiliary::Lookup_2D::Lookup_2D(const VectorXd& reference_1d,
	const VectorXd& reference_2d,
	const MatrixXd& table_data_,
	bool extrapolation) {
	isextrapolation = extrapolation;


	Preprocess();
}

void mathauxiliary::Lookup_2D::GetOutput(double& output,const double& target) {

	index_sequence_.col(0) = BinarySearchVector(true, reference_1d_, target);
	index_sequence_.col(1) = BinarySearchVector(true, reference_2d_, target);
	output = LinearInterpolation2D(table_data_, index_sequence_.col(0), index_sequence_.col(1), reference_1d_, reference_2d_, target);
}

void mathauxiliary::Lookup_2D::Preprocess() {



}


mathauxiliary::Lookup_2D::~Lookup_2D() {


}
