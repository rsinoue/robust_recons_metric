/*
 * sensors.h
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "ros/ros.h"
#include "me_definitions.h"
#include "conversion.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace ME::Types;
using namespace ME::Conversion;
namespace ME {

	class Sensor {
		// This class storages the actual and previous 3D values of the sensors (raw, LPF, estimated predicted).
		// A low-pass filter is used to filtering the raw signals. The low-pass filtered data is saved in the filtering variable.
		// The others variables are only used to storage the data.
	  protected:
		int seq;
		Vector3axes raw;
		Vector3axes rawPrevious;
		double t, tPrevious, dt;
		string name;
		double alphaLPF;
		bool isSensorStarted_;
		bool isNewData_;
		int contStart;
		int contStartSize;
	  public:
		Sensor();
		bool isSensorStarted(void);
		bool isNewData(void);
		int getDataSeq(void);
		double getSampleTime(void);
		Vector3axes getDataRaw(void);
		Vector3axes getDataRawPrevious(void);
		Vector3axes getDataFiltered(void);
		Vector3axes getDataFilteredPrevious(void);
		string getSensorName(void) { return name;};
		void setSensorName(string nameValue) { name = nameValue;};
		void setAlphaLPF(double alphaLPFvalue);
		void setDataSeq(int seqValue);
		virtual void setDataRaw(Vector3axes rawValue, double tValue, int seqValue);
		void setisSensorStarted(bool Value);
		void setisNewData(bool Value);
	}; // class Sensor


	class MAG: public Sensor {
		bool isInsideBounds_;
		Vector3axes globalValue;
		double declination;
		double bounds;
		bool autoMagFieldEst_;
	  public:
		MAG();
		bool isInsideBounds(void);
        Vector3axes getGlobalValue(void);
        double getYaw(double rollAccel, double pitchAccel);
    	void setBoundsPerc(double boundsPercValue);
        void setGlobalValue(Vector3axes globalVector);
        void setDataRaw (Vector3axes rawValue,double tValue,int seqValue);
		void setAutoMagFieldEst(bool Value);
	}; // class MAG: public Sensor

	class ACC: public Sensor {
		bool isInsideBounds_;
		Vector3axes globalValue;
		double bounds;
		bool autoGravEst_;
	  public:
		ACC();
		bool isInsideBounds(void);
		Vector3axes getGlobalValue(void);
        double getRoll(void);
        double getPitch(void);
    	void setBoundsPerc(double boundsPercValue);
        void setGlobalValue(Vector3axes globalVector);
		void setDataRaw (Vector3axes rawValue,double tValue,int seqValue);
		void setAutoGravEst(bool Value);
	}; // class ACC: public Sensor

	class GYRO: public Sensor {

	  public:
		GYRO();
	}; // class GYRO: public Sensor

	class IMU {
		int seq;
		VectorQuat orientation;
		Vector3axes angles;
	  public:
		IMU();
		int getDataSeq(void);
		VectorQuat getIMUOrientation(void);
		Vector3axes getIMUAngles(void);
		void setDataSeq(int seqValue);
		void setIMUOrientation(VectorQuat orientationValue);
	};

	class GPS {
		int seq;
		Vector3axes llh0;
		Vector3axes llh;
		Vector3axes llhPrevious;
		Vector3axes positionECEF;
		Vector3axes position;
		Vector3axes positionPrevious;
		double magDec;
		double direction;
		double speed;
		Matrix3d RmagDec;
		int status;
		bool isGPSStarted_;
		bool isNewData_;
	  public:


		GPS();
		GPS(Vector3axes llhValue, Vector3axes positionValue, double magDecValue);
		bool isGPSStarted(void);
		bool isNewData(void);
		int getDataSeq(void);
		Vector3axes getDataLLH(void);
		Vector3axes getStartPosition(void);
		Vector3axes getDataPosition(void);
		double getDirection(void);
		double getSpeed(void);
		void setDataSeq(int seqValue);
		void setMagDec(double magDecValue);
		void setDataLLH(Vector3axes llhValue, int statusValue) ;
		void setStartPosition(Vector3axes llh0Value);
		void setisGPSStarted(bool Value);
		void setisNewData(bool Value);
		void setDirection(double directionValue);
		void setSpeed(double speedValue);
	};

	class SLAM {
		int seq;
		Vector3axes position;
		Vector3axes positionRotated;
		Vector3axes velocityRotated;
		Vector3axes positionRotatedPrevious;
		double velMag;
		double velMagFilt;
		double alphaLPF;
		Vector3axes positionScaled;
		Vector3axes positionScaledPrevious;
		VectorQuat orientation;
		VectorQuat orientationRotated;
		VectorQuat orientationRotatedPrevious;

		VectorQuat orientationOffset;
		Matrix3d ROffset;
        Vector3axes positionOffset;

		VectorQuat CGq;    // Global to Camera quaternion
		Vector3axes GCd;   // Camera to Global translation
		Matrix3d GCR;      // Camera to Global rotation matrix

		double t, tPrevious, dt;

		double scale;

		bool isSLAMStarted_;
		bool isNewData_;
		bool isOffsetOrientationSet_;
		bool isOffsetPositionSet_;
		bool isOrientationRunning_;
		bool isPositionRunning_;
		bool isScaleSet_;
	  public:
		SLAM();
		bool isSLAMStarted(void);
		bool isNewData(void);
		bool isOffsetOrientationSet(void);
		bool isOffsetPositionSet(void);
		bool isOrientatioRunning(void);
		bool isPositionRunning(void);
		bool isScaleSet(void);
		int getDataSeq(void);
		double getSampleTime(void);
		Vector3axes getDataPosition(void);
		VectorQuat getDataOrientation(void);
		Vector3axes getDataPositionRaw(void);
		VectorQuat getDataOrientationRaw(void);
		Vector3axes getScaledPosition(void);
		double getVelMagFilt(void);
		void setScaleSLAM(double scaleValue);
		void setDataSeq(int seqValue);
		void setisOffsetOrientationSet(bool value);
		void setisOffsetPositionSet(bool value);
//		void setisOrientationRunning(bool value);
//		void setisPositionRunning(bool value);
		void setisSLAMStarted(bool Value);
		void setisNewData(bool Value);
		void setisScaleSet(bool Value);
		void setCamera2GlobalTransformation(VectorQuat orientationValue, Vector3axes displacementValue);
		void setDataSLAM(VectorQuat orientationValue, Vector3axes positionValue,double tValue,int seqValue);
		void setOrientationOffset(VectorQuat orientationREF);
		void setPositionOffset(Vector3axes positionREF);
	};

} // namespace ME

#endif // SENSORS_H

