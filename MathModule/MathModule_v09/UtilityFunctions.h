#pragma once
#include <Eigen\Dense>
#define _USE_MATH_DEFINES // for C++
#include <math.h>
using namespace std;
using namespace Eigen;
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
}