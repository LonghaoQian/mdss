#pragma once
using namespace Eigen;
namespace RungeKuttaFamily {
	enum SolverType {
		DORMANDPRINCE,
		RUNGKUTTA45,
		EULER1ST
	};
	// Load Butchertableau for different methods
	Matrix<double, 9, 8>  InitbutchertableauDORMANDPRINCE();
	Matrix<double, 5, 5>  InitbutchertableauRK4();
	Matrix<double, 2, 2>  InitbutchertableauEuler1();
	void LoadButcherTableau(const SolverType& solver, 
							MatrixXd& table, 
							VectorXd& updatecoefficient1, 
							VectorXd& updatecoefficient2,
							unsigned int& num_of_k);
}

