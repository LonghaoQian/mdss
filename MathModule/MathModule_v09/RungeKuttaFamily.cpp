#include "stdafx.h"
#include "RungeKuttaFamily.h"
Matrix<double, 9, 8> RungeKuttaFamily::InitbutchertableauDORMANDPRINCE()
{
	Matrix<double, 9, 8> DORMANDPRINCE_butchertableau;
	DORMANDPRINCE_butchertableau.setZero(9, 8);
	//Dormand Prince 方法参数，除以浮点型的时候要注意加小数点.0(需要记录）
	//Reference: https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method
	double t21 = 1.0 / 5.0;
	double t22 = 1.0 / 5.0;
	double t31 = 3.0 / 10.0;
	double t32 = 3.0 / 40.0;
	double t33 = 9.0 / 40.0;
	double t41 = 4.0 / 5.0;
	double t42 = 44.0 / 45.0;
	double t43 = -56.0 / 15.0;
	double t44 = 32.0 / 9.0;
	double t51 = 8.0 / 9.0;
	double t52 = 19372.0 / 6561.0;
	double t53 = -25360.0 / 2187.0;
	double t54 = 64448.0 / 6561.0;
	double t55 = -212.0 / 729.0;
	double t61 = 1.0;
	double t62 = 9017.0 / 3168.0;
	double t63 = -355.0 / 33.0;
	double t64 = 46732.0 / 5247.0;
	double t65 = 49.0 / 176.0;
	double t66 = -5103.0 / 18656.0;
	double t71 = 1.0;
	double t72 = 35.0 / 384.0;
	double t73 = 0.0;
	double t74 = 500.0 / 1113.0;
	double t75 = 125.0 / 192.0;
	double t76 = -2187.0 / 6784.0;
	double t77 = 11.0 / 84.0;

	double t82 = 35.0 / 384.0;
	double t83 = 0.0;
	double t84 = 500.0 / 1113.0;
	double t85 = 125.0 / 192.0;
	double t86 = -2187.0 / 6784.0;
	double t87 = 11.0 / 84.0;
	double t88 = 0.0;
	double t92 = 5179.0 / 57600.0;
	double t93 = 0.0;
	double t94 = 7571.0 / 16695.0;
	double t95 = 393.0 / 640.0;
	double t96 = -92097.0 / 339200.0;
	double t97 = 187.0 / 2100.0;
	double t98 = 1.0 / 40.0;

	DORMANDPRINCE_butchertableau(1, 0) = t21;
	DORMANDPRINCE_butchertableau(1, 1) = t22;

	DORMANDPRINCE_butchertableau(2, 0) = t31;
	DORMANDPRINCE_butchertableau(2, 1) = t32;
	DORMANDPRINCE_butchertableau(2, 2) = t33;

	DORMANDPRINCE_butchertableau(3, 0) = t41;
	DORMANDPRINCE_butchertableau(3, 1) = t42;
	DORMANDPRINCE_butchertableau(3, 2) = t43;
	DORMANDPRINCE_butchertableau(3, 3) = t44;

	DORMANDPRINCE_butchertableau(4, 0) = t51;
	DORMANDPRINCE_butchertableau(4, 1) = t52;
	DORMANDPRINCE_butchertableau(4, 2) = t53;
	DORMANDPRINCE_butchertableau(4, 3) = t54;
	DORMANDPRINCE_butchertableau(4, 4) = t55;

	DORMANDPRINCE_butchertableau(5, 0) = t61;
	DORMANDPRINCE_butchertableau(5, 1) = t62;
	DORMANDPRINCE_butchertableau(5, 2) = t63;
	DORMANDPRINCE_butchertableau(5, 3) = t64;
	DORMANDPRINCE_butchertableau(5, 4) = t65;
	DORMANDPRINCE_butchertableau(5, 5) = t66;

	DORMANDPRINCE_butchertableau(6, 0) = t71;
	DORMANDPRINCE_butchertableau(6, 1) = t72;
	DORMANDPRINCE_butchertableau(6, 2) = t73;
	DORMANDPRINCE_butchertableau(6, 3) = t74;
	DORMANDPRINCE_butchertableau(6, 4) = t75;
	DORMANDPRINCE_butchertableau(6, 5) = t76;
	DORMANDPRINCE_butchertableau(6, 6) = t77;

	DORMANDPRINCE_butchertableau(7, 1) = t82;
	DORMANDPRINCE_butchertableau(7, 2) = t83;
	DORMANDPRINCE_butchertableau(7, 3) = t84;
	DORMANDPRINCE_butchertableau(7, 4) = t85;
	DORMANDPRINCE_butchertableau(7, 5) = t86;
	DORMANDPRINCE_butchertableau(7, 6) = t87;
	DORMANDPRINCE_butchertableau(7, 7) = t88;

	DORMANDPRINCE_butchertableau(8, 1) = t92;
	DORMANDPRINCE_butchertableau(8, 2) = t93;
	DORMANDPRINCE_butchertableau(8, 3) = t94;
	DORMANDPRINCE_butchertableau(8, 4) = t95;
	DORMANDPRINCE_butchertableau(8, 5) = t96;
	DORMANDPRINCE_butchertableau(8, 6) = t97;
	DORMANDPRINCE_butchertableau(8, 7) = t98;

	return DORMANDPRINCE_butchertableau;

}
