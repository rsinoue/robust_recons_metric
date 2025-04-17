/*
 * conversion.cpp
 *
 *  Created on: 29 Apr 2016
 *      Author: Roberto Santos Inoue
 */

#include "conversion.h"

namespace ME {

	Vector3axes Conversion::lowPassFilter(Vector3axes vector, Vector3axes vectorPrevious, const double alpha){
		Vector3axes vectorFiltered = alpha*vectorPrevious + (1-alpha)*vector;

		return vectorFiltered;
	}

	double Conversion::lowPassFilter(double value, double valuePrevious, const double alpha){
		double valueFiltered = alpha*valuePrevious + (1-alpha)*value;

		return valueFiltered;
	}

	double Conversion::filterEval(Vector2d b,Vector2d a, double x, double xPrevious, double yPrevious){

		double y = (b(0)*x + b(1)*xPrevious - a(1)*yPrevious)/a(0);

		return y;
	}

	VectorQuat Conversion::quaternionPropagation(Vector3axes omega, VectorQuat q1, const float dt) {
		Vector3axes w =0.5*omega*dt;
		Matrix4d OMEGA = omegaMatrix(w);
		double wAbs = w.norm();
		Matrix4d I4;
		I4 = I4.Identity();
		VectorQuat q2 = (cos(wAbs)*I4 + (sin(wAbs)/wAbs)*OMEGA)*q1;

		return q2;
	}

	Vector3axes Conversion::insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes Gv, Vector3axes ge, const float dt, bool isStationary) {
		Matrix3d GLrot = quat2dcmZYX(LGq).transpose();
		if (!isStationary) {
			Vector3axes f = GLrot*La + ge;
			Gv = f*dt + Gv;
		}
		else {
			Gv = Gv.Zero();
		}
		return Gv;
	}

	Vector3axes Conversion::insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes Gv, Vector3axes ge, const float dt) {
		Matrix3d GLrot = quat2dcmZYX(LGq).transpose();
		Vector3axes f = GLrot*La + ge;
		Gv = f*dt + Gv;
		return Gv;
	}

	Vector3axes Conversion::insPropagation(Vector3axes Gv, Vector3axes Gp, const float dt) {
		Gp = dt*Gv + Gp;

		return Gp;
	}

	Matrix4d Conversion::omegaMatrix(Vector3axes omega) {
		Matrix4d OMEGA;
		OMEGA <<  0,       -omega(0), -omega(1), -omega(2),
				  omega(0), 0,         omega(2), -omega(1),
				  omega(1),-omega(2),  0,         omega(0),
				  omega(2), omega(1), -omega(0),  0;

		return OMEGA;
	}

	VectorQuat Conversion::angle2quatZYX(double yaw, double pitch, double roll){
		VectorQuat q; // q = [w, x, y, z]'
		q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
		q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
		q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
		q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);

		return q;
	}

	Vector3axes Conversion::quat2angleZYX(VectorQuat q){

		Vector3axes Angles;

		Angles(0) = atan2( 2*(q(2)*q(3) + q(0)*q(1)) , (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)) ); // Roll angle - axis X
		Angles(1) = asin( -2*(q(1)*q(3) -q(0)*q(2)) );                                                    // Pitch angle - axis Y
		Angles(2) = atan2( 2*(q(1)*q(2) + q(0)*q(3)) , q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3) );   // Yaw angle - axis Z

		return Angles;
	}


	Matrix3d Conversion::quat2dcmZYX(VectorQuat q) {
		Matrix3d R;
		R << q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)+q(0)*q(3)),             		   2*(q(1)*q(3)-q(0)*q(2)),
			 2*(q(1)*q(2)-q(0)*q(3)),         	      q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3),     2*(q(2)*q(3)+q(0)*q(1)),
			 2*(q(1)*q(3)+q(0)*q(2)),                 2*(q(2)*q(3)-q(0)*q(1)),                     q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);

		return R;
	}

	Matrix3d Conversion::crossMatrix(Vector3axes v){

		Matrix3d M;

		M << 0,   -v(2), v(1),
			 v(2), 0,   -v(0),
			-v(1), v(0), 0;

		return M;
	}

	VectorQuat Conversion::deltaAngle2quat(Vector3axes deltaAngle, VectorQuat q) {
		VectorQuat s;
		VectorQuat deltaq;
		Vector3d deltaqVector = deltaAngle/2;
		double deltaqAux = deltaqVector.transpose()*deltaqVector;



		if (deltaqAux >= 1) {
			deltaq(0) = sqrt(1-deltaqAux);
		}
		else {
			deltaq(0) = 1/sqrt(1+deltaqAux);
			deltaq.segment(1,3) = deltaq(0)*deltaqVector;
		}
		s = quatmultiply(deltaq,q);

		return s;
	}

	VectorQuat Conversion::quatmultiply(VectorQuat p, VectorQuat q) {
		Matrix4d  P;
		VectorQuat s;

		P << p(0), -p(1), -p(2), -p(3),
			 p(1),  p(0), -p(3),  p(2),
			 p(2),  p(3),  p(0), -p(1),
			 p(3), -p(2),  p(1),  p(0);

		s = P*q;

		return s;
	}

	VectorQuat Conversion::quatconj(VectorQuat q) {

		VectorQuat s(q(0),-q(1),-q(2), -q(3));
		return s;
	}

	Vector3d Conversion::geodetic2ecef(Vector3d llh){
		double lat = llh(0);
		double lon = llh(1);
		double h = llh(2);

		// World Geodetic System 1984
		// b = 6356752.31424518; SemiminorAxis [m]
		double a = 6378137; // SemimajorAxis [m]
		double e = 0.0818191908426215; // Eccentricity
		double e2 = e*e;

		double RN = a/(pow((1-e2*pow(sin(lat),2)),0.5));

		Vector3d pe;

		pe(0) = (RN + h)*cos(lat)*cos(lon);    // x
		pe(1) = (RN + h)*cos(lat)*sin(lon);    // y
		pe(2) = (RN*(1-e2)+h)*sin(lat);        // z

		return pe;
	}

	Vector3d Conversion::ecef2ned(Vector3d pe, Vector3d llh0){
		double lat0 = llh0(0);
		double lon0 = llh0(1);
		double h0 = llh0(2);

		Matrix3d RTE;
		RTE << -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),   cos(lat0),
			   -sin(lon0),            cos(lon0),   0,
			   -cos(lat0)*cos(lon0), -cos(lat0)*sin(lon0),  -sin(lat0);

		Vector3d pe0, pt;

		pe0 = geodetic2ecef(llh0);

		pt = RTE*(pe - pe0);

		return pt;
	}

	bool Conversion::isInsideBounds(Vector3axes globalValue, Vector3axes Value, double boundsPercentage, string nameBounds){

		bool output = false;
	    double vecNorm = globalValue.norm();
	    //cout << nameBounds.c_str() << " Bounds function" << endl;

	    if (vecNorm > 0) {
	    	double absError = abs(Value.norm()/vecNorm - 1);
	    	if (absError <= boundsPercentage) {
	    		output = true;
	    		//cout << nameBounds.c_str() << " Inside Bounds" << endl;
	    	}
		    else {
	    		// cout << nameBounds.c_str() <<" Outside Bounds" << endl;
//		    	ROS_WARN("Sensor %s is outside bounds %f. Vector value = [%f,%f,%f]'. Global vector value = [%f,%f,%f]'",nameBounds.c_str(),boundsPercentage,Value(0),Value(1),Value(2),globalValue(0),globalValue(1),globalValue(2));
		    }
	    }
	    else {
	    	// cout << nameBounds.c_str() << "vecNorm == 0" << endl;
//	    	ROS_WARN("Sensor %s is with Global Value equal to zero. Bounds = %f. Vector value = [%f,%f,%f]'. Global vector value = [%f,%f,%f]'",nameBounds.c_str(),boundsPercentage,Value(0),Value(1),Value(2),globalValue(0),globalValue(1),globalValue(2));
	    }
	    return output;
	}

	double Conversion::acc2roll(Vector3axes acc) {
		// Applitcation note AN3182
		// The roll sign was changed
    	double rollAccel = -atan2(acc(1),sqrt(pow(acc(0),2)+ pow(acc(2),2)));
    	return rollAccel;
	}

	double Conversion::acc2pitch(Vector3axes acc) {
		// Application note AN3182
		double pitchAccel = atan2(acc(0),sqrt(pow(acc(1),2) + pow(acc(2),2)));
    	return pitchAccel;
	}

	double Conversion::mag2yaw(Vector3axes mag, double rollAccel, double pitchAccel) {
    	double yawMag = atan2(mag(2)*sin(rollAccel) -(mag(1)*cos(rollAccel)), mag(0)*cos(pitchAccel) + mag(1)*sin(pitchAccel)*sin(rollAccel) + mag(2)*sin(pitchAccel)*cos(rollAccel) );
    	return yawMag;
	}

	double Conversion::mag2yaw(Vector3axes mag) {
		double rollAccel = 0;
		double pitchAccel = 0;
    	double yawMag = atan2(mag(2)*sin(rollAccel) -(mag(1)*cos(rollAccel)), mag(0)*cos(pitchAccel) + mag(1)*sin(pitchAccel)*sin(rollAccel) + mag(2)*sin(pitchAccel)*cos(rollAccel) );
    	return yawMag;
	}

} // namespace ME


