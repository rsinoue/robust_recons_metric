/*
 * motion_estimation.h
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef MOTION_ESTIMATION_H
#define MOTION_ESTIMATION_H

#include "motion_estimation.h"
#include "me_definitions.h"
#include "sensors.h"
#include "conversion.h"
#include <iostream>
#include <fstream>


using namespace ME::Conversion;

namespace ME {

	class MotionEstimation {
	    VectorQuat LGqpHat;
		VectorQuat LGqHat;
		VectorQuat LGqSLAM;
		Vector3axes anglesHat;
		Vector3axes danglesHat;
		Vector3axes anglesGyro;
		Vector3axes GppHat;
		Vector3axes GpHat;
		Vector3axes GvHat;
		Vector3axes GvpHat;
		Vector3axes GvpHatPrevious;

		Vector3axes omegagpHat;
		Vector3axes omegagHat;
		Vector3axes bgpHat;
		Vector3axes bgHat;

		Vector3axes apHat;
		Vector3axes apHatPrevious;
		Vector3axes aHat;
		Vector3axes baHat;
		Vector3axes bapHat;

		Vector3axes varg;
		Vector3axes varbg;
		Vector3axes vara;
		Vector3axes varba;
		Vector3axes varm;
		Vector3axes vareSLAM;
		Vector3axes varpSLAM;
		Vector3axes varpGPS;


		double apHatMagFilt1Previous;
		double apHatMagFilt1;
		double apHatMagFilt2;

		Vector3axes aAcc;
		Vector3axes ge;
		Matrix3d Lambdaa;

		Vector3axes mMag;
		Vector3axes me;

		Vector3axes omegaGyro;
		Matrix3d Lambdag;

		Vector3axes GpSLAM;
		Vector3axes GpGPS;

		Vector2d bHP, aHP, bLP, aLP;

		Vector3axes stateMarkov;

		int yawOption;

		double dt;
		double INSthreshold;
		double posGPSSLAMThreshold;

		double alphaLPF;

		bool isMagAvailable_;
		bool isAccAvailable_;
		bool isSLAMAvailable_;
		bool isSLAMOrientationAvailable_;
		bool isSLAMPositionAvailable_;
		bool isGPSAvailable_;

		bool isINSEnabled_;
		bool isGyroEnabled_;
		bool isAccelEnabled_;
		bool isMagEnabled_;
		bool isSLAMEnabled_;
		bool isSLAMOrientationEnabled_;
		bool isSLAMPositionEnabled_;
		bool isGPSEnabled_;

		bool isGPSSLAMThresholdEnabled_;

		bool isMotionEstimationStarted_;
		bool isAttitudeStarted_;
		bool isPositionStarted_;

		bool isStationary_;

	  public:
		MotionEstimation();

		Vector3axes getStateMarkov(void);
		VectorQuat getEstOrientation(void);
		Vector3axes getEstPosition(void);
		Vector3axes getEstAngularVel(void);
		Vector3axes getEstLinearVel(void);
		Vector3axes getEstAngles(void);
		Vector3axes getGyroAngles(void);
		Vector3axes getBiasGyro(void) ;
		Vector3axes getBiasAcc(void);
		Vector3axes getAccCorrected(void);
		double getSampleTime (void);
		double getPosGPSSLAMThreshold (void);
		Vector3axes getEarthGravity(void);
		Vector3axes getEarthMagneticField(void);
		MatrixF getMatrixF(void);
		MatrixG getMatrixG(void);
		MatrixH getMatrixH(void);
		MatrixR getMatrixR(void);

		int getYawOption(void);

		void setYawOption(int yawOptionValue);

		void setSensorObservationOptions(bool gyro, bool acc, bool mag, bool slam, bool gps);

		void setisINSEnabled(bool ins);
		void setisGyroEnabled(bool gyro);
		void setisAccelEnabled(bool acc);
		void setisMagEnabled(bool mag);
		void setisSLAMEnabled(bool slam);
		void setisSLAMOrientationEnabled(bool slam);
		void setisSLAMPositionEnabled(bool slam);
		void setisGPSEnabled(bool gps);

		void setisGPSSLAMThresholdEnabled(bool value);

		void setisAttitudeStarted(bool Value);
		void setisPositionStarted(bool Value);
		void setisMagAvailable(bool Value);
		void setisAccAvailable(bool Value);
		void setisSLAMAvailable(bool orientationValue,bool positionValue);
		void setisGPSAvailable(bool Value);
		void setisMotionEstimationStarted(bool Value);

		void setGyroVar(Vector3axes gyroVarValue, Vector3axes gyroBiasVarValue);
		void setAccVar(Vector3axes accVarValue, Vector3axes accBiasVarValue);
		void setMagVar(Vector3axes magVarValue);
		void setSLAMVar(Vector3axes eSLAMVarValue, Vector3axes pSLAMVarValue);
		void setGPSVar(Vector3axes pGPSarValue);


		void setAccValue(Vector3axes aAccValue);
		void setMagValue(Vector3axes mMagValue);
		void setGyroValue(Vector3axes omegaGyroValue);
		void setGPSValue(Vector3axes GpGPSValue);
		void setSLAMValue(VectorQuat LGqSLAMValue, Vector3axes GpSLAMValue);

		void setINSthreshold(double INSthresholdValue);
		void setPosGPSSLAMThreshold(double posGPSSLAMThresholdValue);

		void setSampleTime (double dtValue);
		void setEarthGravity(Vector3axes geValue);
		void setEarthMagneticField(Vector3axes geValue);
		void setAlphaLPF(double alphaValue);

		void setMatrixTaug(Matrix3d LambdagValue);
		void setMatrixLambdaa(Matrix3d LambdaaValue);

		void setButterHPFilter(Vector2d bHPValue, Vector2d aHPValue);
		void setButterLPFilter(Vector2d bLPValue, Vector2d aLPValue);

		void startAttitude(double roll, double pitch, double yaw);
		void startPosition(Vector3axes position);
		bool isAttitudeStarted(void);
		bool isPositionStarted(void);
		bool isMotionEstimationStarted(void);

		bool isINSEnabled(void);
		bool isGyroEnabled(void);
		bool isAccelEnabled(void);
		bool isMagEnabled(void);
		bool isSLAMEnabled(void);
		bool isSLAMOrientationEnabled(void);
		bool isSLAMPositionEnabled(void);
		bool isSLAMAvailable(void);
		bool isSLAMOrientationAvailable(void);
		bool isSLAMPositionAvailable(void);

		bool isGPSEnabled(void);

		bool isGPSSLAMThresholdEnabled(void);


		void propagation(void);

		VectorObser observationError(void);
		void state2variables(VectorState x);
		MatrixF montionSys15Matrices(VectorQuat LGq, Vector3axes omega, Vector3axes La, const float dt);
		MatrixG montionSys15Matrices(VectorQuat LGq, const float dt);
		MatrixH montionSys15Matrices(VectorQuat LGq, Vector3axes Gme, Vector3axes Gge);
//		MatrixH montionSys15Matrices(VectorQuat LGq, Vector3axes Gme, Vector3axes Gge, int option);
	};

} // namespace ME

#endif // MOTION_ESTIMATION_H

