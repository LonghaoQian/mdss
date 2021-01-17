// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

#ifndef PCH_H
#define PCH_H

// TODO: add headers that you want to pre-compile here
#include <iostream>

// This file is being reused under Unix, so check OS for filepath separator
#if defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64)
#include <Eigen\Core>
#include <Eigen\Geometry>
#else
#include <Eigen/Core>
#include <Eigen/Geometry>
#endif
#include <vector>
#define _USE_MATH_DEFINES // for C++
#include <math.h>
#include <string.h>
#include <map>
#include <fstream>
#include <functional>
#include <memory>
#endif //PCH_H
