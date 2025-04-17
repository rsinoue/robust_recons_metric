/*
 * conversion.h
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef CONVERSION_H
#define CONVERSION_H

#include "ros/ros.h"
#include "me_definitions.h"
using namespace std;
using namespace ME::Types;



namespace ME {

	namespace Conversion {
		Vector3axes lowPassFilter(Vector3axes vector, Vector3axes vectorPrevious, const double alpha);
		double lowPassFilter(double value, double valuePrevious, const double alpha);
		double filterEval(Vector2d b,Vector2d a, double x, double xPrevious, double yPrevious);
		VectorQuat quaternionPropagation(Vector3axes omega, VectorQuat q1, const float dt);
		Vector3axes insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes Gv, Vector3axes ge, const float dt, bool isStationary);
		Vector3axes insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes Gv, Vector3axes ge, const float dt);
		Vector3axes insPropagation(Vector3axes Gv, Vector3axes Gp, const float dt);
		Matrix4d omegaMatrix(Vector3axes omega);
		VectorQuat angle2quatZYX(double yaw, double pitch, double roll);
		Vector3axes quat2angleZYX(VectorQuat q);
		Matrix3d quat2dcmZYX(VectorQuat q);
		Matrix3d crossMatrix(Vector3axes v);
		VectorQuat deltaAngle2quat(Vector3axes deltaAngle, VectorQuat q1);
		VectorQuat quatmultiply(VectorQuat q1, VectorQuat q2);
		VectorQuat quatconj(VectorQuat q);
		Vector3d geodetic2ecef(Vector3d llh);
		Vector3d ecef2ned(Vector3d pe, Vector3d llh0);
		bool isInsideBounds(Vector3axes globalValue, Vector3axes Value, double boundsPercentage, string nameBounds);
		double acc2roll(Vector3axes acc);
		double acc2pitch(Vector3axes acc);
		double mag2yaw(Vector3axes mag, double rollAccel, double pitchAccel);
		double mag2yaw(Vector3axes mag);
	}
} // namespace ME

#endif // CONVERSION_H
