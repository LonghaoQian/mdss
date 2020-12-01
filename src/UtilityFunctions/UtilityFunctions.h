#pragma once
#include <Eigen\Dense>
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <vector>
using namespace Eigen;
using std::string;
using std::vector;
namespace mathauxiliary {

	enum eulerindex {
		EULER_ROLL = 0,
		EULER_PITCH,
		EULER_YAW
	};

	enum vectorindex {
		VECTOR_X = 0,
		VECTOR_Y,
		VECTOR_Z
	};

	enum quaterionindex {
		QUAT_W = 0,
		QUAT_X,
		QUAT_Y,
		QUAT_Z
	};

	/*----- attitude kinematics ------*/
	// the function maps a cross-product matrix back to the vector
	Vector3d Veemap(const Matrix3d& cross_matrix);
	// the lie algebra of  a vector, or the cross-product matrix of a vector
	Matrix3d Hatmap(const Vector3d& vector);
	// get the Euler angles by a given quaternion
	Vector3d GetEulerAngleFromQuaterion(const Vector4d& quaterion);
	// given a set of Euler angles, compute the corresponding quaternion
	Vector4d GetQuaterionFromRulerAngle(const Vector3d& Euler);
	// the L auxiliary matrix of a quaternion
	Matrix<double, 3, 4> GetLmatrixFromQuaterion(const Vector4d& quaterion);
	// the R auxiliary matrix of a quaternion
	Matrix<double, 3, 4> GetRmatrixFromQuaterion(const Vector4d& quaterion);
	// compute the rotation matrix corresponding to a quaternion
	Matrix3d GetR_IBFromQuaterion(const Vector4d& quaterion);
	// distribute the columns of a rotation matrix into a 9 by 1 vector 
	Matrix<double, 9, 1> ConvertRotationMatrixToVector(const Matrix<double,3,3>& R);
	// form a rotation matrix with a 9 by 1 vector by columns
	Matrix<double, 3, 3> ConvertVectorToRotationMatrix(const Matrix<double, 9,1>& v);
	void ConvertVectorToRotationMatrix(const Matrix<double, 9, 1>& v, Matrix3d& R);
	// output the rotation matrix based on roll, pitch, and yaw angles
	// the rotation matrices obtained here are R21, where 1 frame is the base frame, 2 is the rotated frame
	Matrix<double, 3, 3> RotationMatrixFromRoll(const double& phi);
	Matrix<double, 3, 3> RotationMatrixFromPitch(const double& theta);
	Matrix<double, 3, 3> RotationMatrixFromYaw(const double& psi);
	void RotationMatrixFromRoll(const double& phi, Matrix3d& R21);
	void RotationMatrixFromPitch(const double& theta, Matrix3d& R21);
	void RotationMatrixFromYaw(const double& psi, Matrix3d& R21);

	/*--------- topology --------------*/
	Vector2i BinarySearchVector(bool isascending,
								const VectorXd& p, 
								const double& target);

	double   LinearInterpolation1D(const VectorXd& data, 
								   int index1, 
								   int index2, 
								   const VectorXd& reference, 
								   const double& target);

	double   LinearInterpolation2D(const MatrixXd& data, 
								   const Vector2i& index_1d, 
								   const Vector2i& index_2d, 
								   const VectorXd& reference_1d,
								   const VectorXd& reference_2d, 
								   const double& target1,
								   const double& target2);

	/*---------------- lookup methods --------------------------*/
	// base class: 
	class LookupInterface {
		// input must be sorted array.
	protected:
		bool isextrapolation; // determine whether extrapolation is used
		virtual void Preprocess() = 0;
	public:
		LookupInterface(bool extrapolation);
		LookupInterface();
		~LookupInterface();
	};

	class Constant :
		public LookupInterface {
	private:
		void Preprocess();
		double value_;
	public:
		void GetOutput(double& output, const double& target);
		double GetOutput(const double& target);
		Constant(double& value);
		~Constant();
	};

	class Linear : 
		public LookupInterface
	{
	private: 
		void Preprocess();
		double slop_;
	public:
		void GetOutput(double& output, const double& target);
		double GetOutput(const double& target);
		Linear();
		Linear(double slop);
		void LoadData(double slop);
		~Linear();

	};

	class Lookup_1D : 
		public LookupInterface
	{
	private:
		int num_of_references_;
		int num_of_data_arrays;
		bool ismulti_;
		VectorXd reference_1d_;
		VectorXd table_data_;
		MatrixXd table_data_multi_;
		Vector2i index_sequence_;
		void Preprocess();
	public:
		Lookup_1D();
		Lookup_1D(const VectorXd& reference_1d,
			const MatrixXd& _data,
			bool extrapolation);
		void GetOutput(double& output, const double& target);
		void GetOutput(VectorXd& output, const double& target);
		double GetOutput(const double& target);
		void LoadTableData(const VectorXd& reference_1d,
						   const MatrixXd& _data,// accept multi cols for multiple ouputs
						   bool extrapolation);
		~Lookup_1D();
	};
	class Lookup_2D :
	public LookupInterface {
	private:
		int num_of_references_1d_;
		int num_of_references_2d_;
		int num_of_tables_;
		VectorXd reference_1d_;
		VectorXd reference_2d_;
		Matrix2i index_sequence_;
		MatrixXd table_data_;
		void Preprocess();
	public:
		Lookup_2D();
		Lookup_2D(const VectorXd& reference_1d, // row reference
				  const VectorXd& reference_2d, // col reference
				  const MatrixXd& table_data_,  // table
				  bool extrapolation);
		void GetOutput(double& output, const double& target1, const double& target2);
		double GetOutput(const double& target1,const double& target2);
		void LoadTableData(const VectorXd& reference_1d,
						   const VectorXd& reference_2d,
						   const MatrixXd& table_data_,
						   bool extrapolation);
		~Lookup_2D();
	};
	/*
	class Lookup_3D {
	private:
	public:
		Lookup_3D();
		void GetOutput(double& output, bool extrapolation);
		~Lookup_3D();
	};
	// TO DO n-D look up*/
	/*---------- saturation function -----------------*/
	void SaturationElementalWise(VectorXd& output, 
								 const VectorXd& upper_limit_,
								 const VectorXd& lower_limit_);

	void SaturationElementalWise(VectorXd& output,
								 const double& upper_limit_,
								 const double& lower_limit_);

	void SaturationElementalWise(double& output, 
								 const double& upper_limit_,
							     const double& lower_limit_);

	void SaturationVector(VectorXd& output, 
						  const double& upper_limit, 
						  const double& lower_limit_);
	double SaturationElementalWise(const double& input,
						 const double& upper_limit_,
						 const double& lower_limit_);
}