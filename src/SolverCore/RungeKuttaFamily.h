#pragma once
#include <Eigen\Dense>
///��ⷽ��������
#define DORMANDPRINCE   0x10
#define RUNGKUTTA45     0x11
#define GEARSMETHOD115  0x12
#define LINEARMULTISTEP 0x13
using namespace Eigen;
namespace RungeKuttaFamily {
	// Butchertableau DORMANDPRINCE;
	Matrix<double, 9, 8>  InitbutchertableauDORMANDPRINCE();
}

