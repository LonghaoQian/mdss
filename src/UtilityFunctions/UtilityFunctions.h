#pragma once
#include <Eigen\Dense>
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <vector>
using namespace Eigen;
using std::string;
using std::vector;
namespace mathauxiliary {

	/*----- attitude kinematics ------*/
	Vector3d Veemap(const Matrix3d& cross_matrix);
	Matrix3d Hatmap(const Vector3d& vector);
	Vector3d GetEulerAngleFromQuaterion(const Vector4d& quaterion);
	Vector4d GetQuaterionFromRulerAngle(const Vector3d& Euler);
	Matrix<double, 3, 4> GetLmatrixFromQuaterion(const Vector4d& quaterion);
	Matrix<double, 3, 4> GetRmatrixFromQuaterion(const Vector4d& quaterion);
	Matrix3d GetR_IBFromQuaterion(const Vector4d& quaterion);
	Matrix<double, 9, 1> ConvertRotationMatrixToVector(const Matrix<double,3,3>& R);
	Matrix<double, 3, 3> ConvertVectorToRotationMatrix(const Matrix<double, 9,1>& v);

	/*--------- topology --------------*/
	Vector2i BinarySearchVector(bool isascending,
								const VectorXd& p, 
								double& target);

	double   LinearInterpolation1D(const VectorXd& data, 
								   int index1, 
								   int index2, 
								   const VectorXd& reference, 
								   double& target);

	double   LinearInterpolation2D(const MatrixXd& data, 
								   const Vector2i& index_1d, 
								   const Vector2i& index_2d, 
								   const VectorXd& reference_1d,
								   const VectorXd& reference_2d, 
								   double& target);

	/*---------------- lookup methods --------------------------*/
	// base class: 
	class LookupInterface {
		// input must be sorted array.
	protected:
		bool isextrapolation; // determine whether extrapolation is used
		vector<bool> isascending;     // flag showing 
		virtual void Preprocess() = 0;
	public:
		virtual void GetOutput() = 0;
		LookupInterface();
		~LookupInterface();
	};
	class Lookup_1D : 
		public LookupInterface
	{
	private:
		int num_of_references_;
		VectorXd reference_1d_;
		VectorXd table_data_;
		Vector2i index_squence_;
		void Preprocess();
	public:
		Lookup_1D(const VectorXd& reference_1d,
			      const VectorXd& _data, 
				  bool extrapolation);
		void GetOutput(double& output, double& target);
		~Lookup_1D();
	};
	class Lookup_2D {
	private:
		int num_of_inputs1;
		int num_of_inputs2;
		VectorXd input_1;
		VectorXd input_2;
		MatrixXd data;
	public:
		Lookup_2D(const VectorXd& _input_1, const VectorXd _input_2, const MatrixXd& data, bool extrapolation);
		void GetOutput(double& output);
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
}