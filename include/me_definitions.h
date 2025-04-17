/*
 * me_definitions.h
 *
 *  Created on: 29 Apr 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef ME_DEFINITIONS_H
#define ME_DEFINITIONS_H

#include "Eigen/Dense"
#include <math.h>

using namespace std;
using namespace Eigen;


namespace ME {
    // Constant variables
	const float d2r = M_PI/180;
	const float r2d = 180/M_PI;
	const float T2G = 10e4;  	// Tesla to Gauss

    namespace Types{
	// System sizes
	const int sizeState = 15;
	const int sizeObser = 9;
	const int sizewNoise = 12;
	const int sizevNoise = 9;
	const int sizeQuat = 4;
	const int size3axes = 3;

	// Eigen types
	typedef Matrix<double, sizeState, 1> VectorState;
	typedef Matrix<double, sizeObser, 1> VectorObser;
	typedef Matrix<double, sizeQuat, 1> VectorQuat;
	typedef Matrix<double, size3axes, 1> Vector3axes;
	typedef Matrix<double, sizeState, sizeState> MatrixF;
	typedef Matrix<double, sizeState, sizewNoise> MatrixG;
	typedef Matrix<double, sizeObser, sizeState> MatrixH;
	typedef Matrix<double, sizeState, sizeState> MatrixP;
	typedef Matrix<double, sizewNoise, sizewNoise> MatrixQ;
	typedef Matrix<double, sizeState, sizeState> MatrixQd;
	typedef Matrix<double, sizevNoise, sizevNoise> MatrixR;
	typedef Matrix<double, sizeState, sizeObser> MatrixK;
    }

} // namespace ME
#endif //ME_DEFINITIONS_H
