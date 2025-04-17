/*
 * sensors.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "sensors.h"

namespace ME {

	Sensor::Sensor() {
		seq = 0;
		raw << 0, 0, 0;
		rawPrevious << 0, 0, 0;
		alphaLPF = 0.99;
		name = "Sensor";
		isSensorStarted_ = false;
		isNewData_ = false;
		contStart = 0;
		contStartSize = 1;
	}

	bool Sensor::isSensorStarted(void){
		return isSensorStarted_;
	}

	bool Sensor::isNewData(void){
		return isNewData_;
	}

	int Sensor::getDataSeq(void){return seq;}
	double Sensor::getSampleTime(void){return dt;}
	Vector3axes Sensor::getDataRaw(void){return raw;}
	Vector3axes Sensor::getDataRawPrevious(void){return rawPrevious;}

	void Sensor::setDataSeq(int seqValue){seq = seqValue;}
	void Sensor::setAlphaLPF(double alphaLPFvalue){alphaLPF = alphaLPFvalue;}
	void Sensor::setDataRaw (Vector3axes rawValue, double tValue, int seqValue){
		isNewData_ = true;
		seq = seqValue;
		rawPrevious = raw;
		raw = rawValue;
		tPrevious = t;
		t = tValue;
		dt = t - tPrevious;
		contStart ++;
		if (!isSensorStarted_ && contStart > contStartSize) {
			isSensorStarted_ = true;
		}
	}
	void Sensor::setisSensorStarted(bool Value){
		isSensorStarted_ = Value;
	}
	void Sensor::setisNewData(bool Value){
		isNewData_ = Value;
	}

	MAG::MAG(): Sensor::Sensor() {
		isInsideBounds_ = false;
		globalValue << 0, 0, 0;
		declination = 0;
		bounds = 0.2;
		autoMagFieldEst_ = true;
		name = " Magnetometer";
		}


	bool MAG::isInsideBounds(void) {

		return isInsideBounds_;
	}

	Vector3axes MAG::getGlobalValue(void){
		return globalValue;
	}

    double MAG::getYaw(double rollAccel, double pitchAccel){
    	return mag2yaw(raw, rollAccel, pitchAccel);
    }

	void MAG::setBoundsPerc(double boundsPercValue) {
		bounds = boundsPercValue;
	}

    void MAG::setGlobalValue(Vector3axes globalVector){
    	globalValue = globalVector;
    }

	void MAG::setDataRaw (Vector3axes rawValue,double tValue,int seqValue){
		isNewData_ = true;
		seq = seqValue;

		rawPrevious = raw;
		raw = rawValue;

		tPrevious = t;
		t = tValue;
		dt = t - tPrevious;

		contStart ++;
		if (!isSensorStarted_ && contStart > contStartSize) {
			isSensorStarted_ = true;
			if (autoMagFieldEst_) {
				globalValue << raw.segment(0,2).norm(), 0, raw(2);
				cout << "Using  me = [" << globalValue.transpose() << "];" << endl;
			}
			else {
				Vector3axes meEst(raw.segment(0,2).norm(), 0, raw(2));
				cout << "Using  me = [" << globalValue.transpose() << "];" << endl;
				cout << "Estimated me = [" << meEst.transpose() << "];" << endl;
			}
		}

		isInsideBounds_ = Conversion::isInsideBounds(globalValue, raw, bounds,name);

	}
	void MAG::setAutoMagFieldEst(bool Value) {
		autoMagFieldEst_ = Value;
	}

	ACC::ACC(): Sensor::Sensor() {
		isInsideBounds_ = false;
		globalValue = globalValue.Zero();
		bounds = 0.2;
		autoGravEst_ = true;
		name = "Accelerometer";
	}

	bool ACC::isInsideBounds(void) {

		return isInsideBounds_;
	}

	void ACC::setBoundsPerc(double boundsPercValue) {
		bounds = boundsPercValue;
	}

	Vector3axes ACC::getGlobalValue(void){
		return globalValue;
	}

    double ACC::getRoll(void){
    	return acc2roll(raw);
    }
    double ACC::getPitch(void){
    	return acc2pitch(raw);
    }

    void ACC::setGlobalValue(Vector3axes globalVector){
    	globalValue = globalVector;
    }

    void ACC::setDataRaw (Vector3axes rawValue,double tValue,int seqValue){
		isNewData_ = true;
		rawPrevious = raw;
		raw = rawValue;

		tPrevious = t;
		t = tValue;
		dt = t - tPrevious;

		contStart ++;
		if (!isSensorStarted_ && contStart > contStartSize) {
			isSensorStarted_ = true;
			if (autoGravEst_) {
			globalValue << 0, 0, raw.norm();
			cout << "Using ge = [" << globalValue.transpose() << "];" << endl;
			}
			else {
			Vector3axes geEst(0,0,raw.norm());
			cout << "Using ge = [" << globalValue.transpose() << "];" << endl;
			cout << "Estimated ge = [" << geEst.transpose() << "];" << endl;
			}
		}
        isInsideBounds_ = Conversion::isInsideBounds(globalValue, raw,bounds,name);
	}

	void ACC::setAutoGravEst(bool Value) {
		autoGravEst_ = Value;
	}

	GYRO::GYRO(): Sensor::Sensor() {
		name = "Gyroscope";
	}

	IMU::IMU() {
		seq = 0;
		orientation = orientation.Zero();
		angles  = angles.Zero();
	}
	int IMU::getDataSeq(void) {
		return seq;
	}
	VectorQuat IMU::getIMUOrientation(void) {
		return orientation;
	}
	Vector3axes IMU::getIMUAngles(void) {
		return quat2angleZYX(orientation);
	}
	void IMU::setDataSeq(int seqValue) {
		seq = seqValue;
	}
	void IMU::setIMUOrientation(VectorQuat orientationValue){
		orientation = orientationValue;
	}

	GPS::GPS() {
		seq = 0;
		llh0 << 0, 0, 0;
		llh << 0, 0, 0;
		llhPrevious << 0, 0, 0;
		position << 0, 0, 0;
		positionPrevious << 0, 0, 0;
		direction = 0;
		speed = 0;
		magDec = 0;
	    RmagDec = RmagDec.Identity();
		isGPSStarted_ = false;
		isNewData_ = false;
	}
	GPS::GPS(Vector3axes llhValue, Vector3axes positionValue, double magDecValue) {
		seq = 0;
		llh0 = llhValue;
		llh = llhValue;
		llhPrevious = llhValue;
		position = positionValue;
		positionPrevious = positionValue;
		direction = 0;
		speed = 0;
		magDec = magDecValue;
		VectorQuat qmagDec = Conversion::angle2quatZYX(magDec,0,0);
		RmagDec = Conversion::quat2dcmZYX(qmagDec);
		isGPSStarted_ = false;
		isNewData_ = false;
	}
	bool GPS::isGPSStarted(void) {
		return isGPSStarted_;
	}
	bool GPS::isNewData(void) {
		return isNewData_;
	}

	int GPS::getDataSeq(void) {
		return seq;
	}

	Vector3axes GPS::getDataLLH(void) {
		return llh;
	}
	Vector3axes GPS::getStartPosition(void) {
		return llh0;
	}
	Vector3axes GPS::getDataPosition(void){
		return position;
	}
	double GPS::getDirection(void) {
		return direction+magDec;
	}
	double GPS::getSpeed(void) {
		return speed;
	}
	void GPS::setDataSeq(int seqValue){
		seq = seqValue;
	}
	void GPS::setMagDec(double magDecValue) {
		magDec = magDecValue;
		VectorQuat qmagDec = Conversion::angle2quatZYX(magDec,0,0);
		RmagDec = Conversion::quat2dcmZYX(qmagDec);
	}
	void GPS::setDataLLH(Vector3axes llhValue, int statusValue) {
        isNewData_ = true;
		status = statusValue;
		if ( status != -1) {
			if (!isGPSStarted_) {
				llh0 = llhValue;
				llhPrevious = llhValue;
				llh = llhValue;
				positionECEF = Conversion::geodetic2ecef(llh);
				position = RmagDec*Conversion::ecef2ned(positionECEF, llh0);
				positionPrevious = position;
				isGPSStarted_ = true;
				cout << "llh0 = [" << setprecision(20) << llh0.transpose()*r2d << "]'" << endl;
			}
			else {
				llhPrevious = llh;
				llh = llhValue;
				positionPrevious = position;
				positionECEF = Conversion::geodetic2ecef(llh);
				position = RmagDec*Conversion::ecef2ned(positionECEF, llh0);
			}
		}
		else {
			//TODO
			ROS_WARN("GPS Status -1");
		}
	}
	void GPS::setStartPosition(Vector3axes llh0Value) {
		llh0 = llh0Value;
	}
	void GPS::setisGPSStarted(bool Value){
		isGPSStarted_ = Value;
	}
	void GPS::setisNewData(bool Value){
		isNewData_ = Value;
	}

	void GPS::setDirection(double directionValue) {
		direction = directionValue;
	}
	void GPS::setSpeed(double speedValue){
		speed = speedValue;
	}


	SLAM::SLAM(){
		orientationRotated << 1, 0, 0, 0;
		orientationRotatedPrevious = orientationRotated;
		positionRotated = positionRotated.Zero();
		velocityRotated = velocityRotated.Zero();
		velMag = 0;
		velMagFilt = 0;
		alphaLPF = 0.9;
		positionRotatedPrevious = positionRotatedPrevious.Zero();
		orientationOffset << 1, 0, 0, 0;
		ROffset = ROffset.Identity();
		positionOffset = positionOffset.Zero();
//		setCamera2GlobalTransformation(angle2quatZYX(90*d2r,0*d2r,90*d2r),Vector3axes::Zero()); // ORB_SLAM2
		setCamera2GlobalTransformation(angle2quatZYX(90*d2r,0*d2r,90*d2r),Vector3axes::Zero()); // LSD_SLAM
		t = 0;
		tPrevious = 0;
		dt = 0.05;
		scale = 0;
		isSLAMStarted_ = false;
		isNewData_ = false;
		isOffsetOrientationSet_ = false;
		isOffsetPositionSet_ = false;
		isOrientationRunning_ = false;
		isPositionRunning_ = false;
		isScaleSet_ =  false;
	}

	bool SLAM::isSLAMStarted(void) {
		return isSLAMStarted_;
	}
	bool SLAM::isNewData(void) {
		return isNewData_;
	}
	bool SLAM::isOffsetOrientationSet(void){
		return isOffsetOrientationSet_;
	}
	bool SLAM::isOffsetPositionSet(void){
		return isOffsetPositionSet_;
	}
	bool SLAM::isOrientatioRunning(void) {
		return isOrientationRunning_;
	}
	bool SLAM::isPositionRunning(void) {
		return isPositionRunning_;
	}
	bool SLAM::isScaleSet(void){
		return isScaleSet_;
	}
	int SLAM::getDataSeq(void) {
		return seq;
	}
	double SLAM::getSampleTime(void){
		return dt;
	}
	Vector3axes SLAM::getDataPosition(void){
		return positionRotated;
	}
	VectorQuat SLAM::getDataOrientation(void){
		return orientationRotated;
	}
	Vector3axes SLAM::getDataPositionRaw(void){
		return position;
	}
	VectorQuat SLAM::getDataOrientationRaw(void){
		return orientation;
	}
	Vector3axes SLAM::getScaledPosition(void){
		return positionScaled;
	}
	double SLAM::getVelMagFilt(void) {
		return velMagFilt;
	}
	void SLAM::setScaleSLAM(double scaleValue){
		scale = scaleValue;
		isScaleSet_ = true;
	}
	void SLAM::setDataSeq(int seqValue){
		seq = seqValue;
	}
	void SLAM::setisOffsetOrientationSet(bool value){
		isOffsetOrientationSet_ = value;
	}
	void SLAM::setisOffsetPositionSet(bool value){
		isOffsetPositionSet_ = value;
	}
//	void SLAM::setisOrientationRunning(bool value) {
//		isOrientationRunning_ = value;
//	}
//	void SLAM::setisPositionRunning(bool value) {
//		isPositionRunning_ = value;
//	}
	void SLAM::setisSLAMStarted(bool Value){
		isSLAMStarted_ = Value;
	}
	void SLAM::setisNewData(bool Value){
		isNewData_ = Value;
	}
	void SLAM::setisScaleSet(bool Value){
		isScaleSet_ = Value;
	}
	void SLAM::setCamera2GlobalTransformation(VectorQuat orientationValue, Vector3axes translationValue) {
		// orientationValue is in the Global frame
		// translationValue is in the Global frame
		// Gp = GCR*Cp + GCd
		GCd = translationValue;
		CGq = orientationValue;
		GCR = quat2dcmZYX(CGq).transpose();
	}
	void SLAM::setDataSLAM(VectorQuat orientationValue, Vector3axes positionValue,double tValue,int seqValue) {
		isNewData_= true;
		if (!isSLAMStarted_) {
			orientation = quatmultiply(CGq,quatmultiply(orientationValue,quatconj(CGq)));
			orientationRotated =  quatmultiply(orientationOffset,orientation);
			orientationRotatedPrevious = orientationRotated;
			position = GCR*positionValue;
			positionRotated = ROffset.transpose()*position;
			positionScaled = positionRotated*scale + GCd + positionOffset;
			positionRotatedPrevious = positionRotated;
			positionScaledPrevious = positionScaled;
			isSLAMStarted_ = true;
			t = tValue;
			tPrevious = t;
//			dt = t - tPrevious;
		}
		else {
			orientationRotatedPrevious = orientationRotated;
			orientation = quatmultiply(CGq,quatmultiply(orientationValue,quatconj(CGq)));
			orientationRotated =  quatmultiply(orientationOffset,orientation);
			positionRotatedPrevious = positionRotated;
			position = GCR*positionValue;
			positionRotated = ROffset.transpose()*position;
			positionScaledPrevious = positionScaled;
			positionScaled = positionRotated*scale + GCd + positionOffset;
			tPrevious = t;
			t = tValue;
			dt = t - tPrevious;
			// TODO change to a variable dt value threshold
			if (dt > 2) {
				isOrientationRunning_ = false;
				isPositionRunning_ = false;
			}
			else {
				isOrientationRunning_  = true;
				if (isScaleSet_)
					isPositionRunning_ = true;
				else
					isPositionRunning_ = false;
			}
		}
		velocityRotated = (positionRotated - positionRotatedPrevious)/dt;
		velMag = velocityRotated.norm();
		velMagFilt = lowPassFilter(velMag,velMagFilt,alphaLPF);
		// cout << "orientationSLAM = [" << orientation.transpose() << "];" << endl;
		// cout << "anglesSLAM = ["  << quat2angleZYX(orientationValue).transpose()*r2d << "];" << endl;
		// cout << "anglesSLAMRotated = ["  << quat2angleZYX(orientation).transpose()*r2d << "];" << endl;
		// cout << "positionSLAM = [" << position.transpose() << "];" << endl;
	}

	void SLAM::setOrientationOffset(VectorQuat orientationREF) {
		orientationOffset = quatmultiply(orientationREF,quatconj(orientation));
		ROffset = quat2dcmZYX(orientationOffset);
//		cout << "orientationOffset = ["  << orientationOffset << "];" << endl;
//		cout << "ROffset = [" << ROffset << "];" << endl;
		isOffsetOrientationSet_ = true;
		//ros::shutdown();
	}
	void SLAM::setPositionOffset(Vector3axes positionREF) {
		positionOffset = positionREF -positionRotated*scale;
		cout << "positionOffset = ["  << positionOffset << "];" << endl;
		isOffsetPositionSet_ = true;
	}
} // namespace ME


