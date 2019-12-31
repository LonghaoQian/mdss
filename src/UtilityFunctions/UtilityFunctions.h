#pragma once
#include <Eigen\Dense>
#define _USE_MATH_DEFINES // for C++
#include <math.h>
using namespace Eigen;
using std::string;
namespace mathauxiliary {
	Vector3d Veemap(const Matrix3d& cross_matrix);
	Matrix3d Hatmap(const Vector3d& vector);
	Vector3d GetEulerAngleFromQuaterion(const Vector4d& quaterion);
	Vector4d GetQuaterionFromRulerAngle(const Vector3d& Euler);
	Matrix<double, 3, 4> GetLmatrixFromQuaterion(const Vector4d& quaterion);
	Matrix<double, 3, 4> GetRmatrixFromQuaterion(const Vector4d& quaterion);
	Matrix3d GetR_IBFromQuaterion(const Vector4d& quaterion);
	Matrix<double, 9, 1> ConvertRotationMatrixToVector(const Matrix<double,3,3>& R);
	Matrix<double, 3, 3> ConvertVectorToRotationMatrix(const Matrix<double, 9,1>& v);
	/*---------------- lookup methods --------------------------*/
	// base class:
	class Lookup {
	protected:
		bool isextrapolation;
		virtual void Preprocess() = 0;
	public:
		virtual void GetOutput() = 0;
		Lookup();
		~Lookup();
	};
	class Lookup_1D : 
		public Lookup 
	{
	private:
		VectorXd input_1;
		VectorXd data;
		void Preprocess();
	public:
		Lookup_1D(VectorXd& input_1, VectorXd& data, bool extrapolation);
		void GetOutput(double& output);
		~Lookup_1D();
	};
	class Lookup_2D {
	private:
		VectorXd input_1;
		VectorXd input_2;
		MatrixXd data;
	public:
		Lookup_2D(VectorXd& input_1, VectorXd input_2, MatrixXd& data, bool extrapolation);
		void GetOutput(double& output);
		~Lookup_2D();
	};
	class Lookup_3D {
	private:
	public:
		Lookup_3D();
		void GetOutput(double& output, bool extrapolation);
		~Lookup_3D();
	};
	// TO DO n-D look up
}