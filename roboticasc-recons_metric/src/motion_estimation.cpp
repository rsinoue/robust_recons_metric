/*
 * motion_estimation.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "motion_estimation.h"

namespace ME {

	MotionEstimation::MotionEstimation() {
		LGqHat = LGqHat.Zero();
		LGqpHat = LGqpHat.Zero();
		anglesHat = anglesHat.Zero();
		danglesHat = danglesHat.Zero();
		anglesGyro = anglesGyro.Zero();
		GppHat = GppHat.Zero();;
		GpHat = GpHat.Zero();
		GvHat = GvHat.Zero();
		GvpHat = GvpHat.Zero();
		GvpHatPrevious = GvpHat.Zero();

		omegagpHat = omegagpHat.Zero();
		omegagHat = omegagHat.Zero();
		bgpHat = omegagHat.Zero();
		bgHat = omegagHat.Zero();

		apHat = apHat.Zero();
		apHatPrevious = apHatPrevious.Zero();
		aHat = aHat.Zero();
		baHat = baHat.Zero();
		bapHat = bapHat.Zero();

		apHatMagFilt1Previous = 0;
		apHatMagFilt1 = 0;
		apHatMagFilt2 = 0;

		aAcc = aAcc.Zero();
		ge = ge.Zero();

		mMag = mMag.Zero();
		me = me.Zero();

		omegaGyro = omegaGyro.Zero();

		GpSLAM = GpSLAM.Zero();
		GpGPS = GpGPS.Zero();

		bHP << 0.999968585060383,  -0.999968585060383; // = bHP.Zero();
		aHP << 1.000000000000000,  -0.999937170120766; // = aHP.Zero();
		bLP << 0.136728735997319,   0.136728735997319; // = bLP.Zero();
		aLP << 1.000000000000000,  -0.726542528005361; // = aLP.Zero();

		stateMarkov = stateMarkov.Zero();

		yawOption = 0;

        dt = 0.01;

	    INSthreshold = 0.01;
	    posGPSSLAMThreshold = 15;

		alphaLPF = 0.99;

		isMagAvailable_ = false;
		isAccAvailable_ = false;
		isSLAMAvailable_ = false;
		isGPSAvailable_ = false;

		isINSEnabled_ = false;
		isGyroEnabled_ = true;
		isAccelEnabled_ = true;
		isMagEnabled_ = true;
		isSLAMEnabled_ = true;
		isSLAMOrientationEnabled_ = true;
		isSLAMPositionEnabled_ = true;
		isGPSEnabled_ = true;
		isGPSSLAMThresholdEnabled_ = false;

		isMotionEstimationStarted_ = false;
		isAttitudeStarted_ = false;
		isPositionStarted_ = false;

		isStationary_ = true;
	}

	Vector3axes MotionEstimation::getStateMarkov(void){
		return stateMarkov;
	}

	VectorQuat MotionEstimation::getEstOrientation(void) {
		return LGqHat;
	}

	Vector3axes MotionEstimation::getEstPosition(void) {
		return GpHat;
	}

	Vector3axes MotionEstimation::getEstAngularVel(void) {
		return danglesHat;
	}

	Vector3axes MotionEstimation::getEstLinearVel(void) {
		return GvHat;
	}

	Vector3axes MotionEstimation::getEstAngles(void) {
		return anglesHat;
	}

	Vector3axes MotionEstimation::getGyroAngles(void) {
			return anglesGyro;
	}

	Vector3axes MotionEstimation::getBiasGyro(void) {
			return bgHat;
	}

	Vector3axes MotionEstimation::getBiasAcc(void) {
			return baHat;
	}

	Vector3axes MotionEstimation::getAccCorrected(void) {
		return apHat;
	}

	double MotionEstimation::getSampleTime (void) {
		return dt;
	}

	double MotionEstimation::getPosGPSSLAMThreshold (void) {
		return posGPSSLAMThreshold;
	}

	Vector3axes MotionEstimation::getEarthGravity(void) {
		return ge;
	}

	Vector3axes MotionEstimation::getEarthMagneticField(void) {
		return me;
	}

	MatrixF MotionEstimation::getMatrixF(void) {
		return montionSys15Matrices(LGqpHat, omegagpHat, apHat, dt);
	}

	MatrixG MotionEstimation::getMatrixG(void) {
		return  montionSys15Matrices(LGqpHat, dt);
	}

	MatrixH MotionEstimation::getMatrixH(void) {
		return  montionSys15Matrices(LGqpHat, me, ge);
	}

	MatrixR MotionEstimation::getMatrixR(void) {
		MatrixR R = MatrixR::Zero();

		if (isSLAMOrientationAvailable_) {
			R.block(0,0,3,3).diagonal() = vareSLAM;
		}
		else if (isMagAvailable_) {
			R.block(0,0,3,3).diagonal() = varm;
		}
		else {
			R.block(0,0,3,3).diagonal() = varm;
		}

		if (isAccAvailable_) {
			R.block(3,3,3,3).diagonal() = vara;
		}
		else {
			R.block(3,3,3,3).diagonal() = vara;
		}

		if (isSLAMPositionAvailable_) {
			R.block(6,6,3,3).diagonal() = varpSLAM;
		}
		else if (isMagAvailable_) {
			R.block(6,6,3,3).diagonal() = varpGPS;
		}
		else {
			R.block(6,6,3,3).diagonal() = varpGPS;
		}
		return R;

	}

	int MotionEstimation::getYawOption(void) {
		return yawOption;
	}

	void MotionEstimation::setYawOption(int yawOptionValue) {
		yawOption = yawOptionValue;
	}

	void MotionEstimation::setSensorObservationOptions(bool gyro, bool acc, bool mag, bool slam, bool gps) {
		isGyroEnabled_ = gyro;
		isAccelEnabled_ = acc;
		isMagEnabled_ = mag;
		isSLAMEnabled_ = slam;
		isGPSEnabled_ = gps;
	}

	void MotionEstimation::setisINSEnabled(bool ins) {
		isINSEnabled_ = ins;
	}

	void MotionEstimation::setisGyroEnabled(bool gyro) {
		isGyroEnabled_ = gyro;
	}

	void MotionEstimation::setisAccelEnabled(bool acc) {
		isAccelEnabled_ = acc;
	}

	void MotionEstimation::setisMagEnabled(bool mag) {
		isMagEnabled_ = mag;
	}

	void MotionEstimation::setisSLAMEnabled(bool slam) {
		isSLAMEnabled_ = slam;
	}

	void MotionEstimation::setisSLAMOrientationEnabled(bool slam) {
		isSLAMOrientationEnabled_ = slam;
	}

	void MotionEstimation::setisSLAMPositionEnabled(bool slam) {
		isSLAMPositionEnabled_ = slam;
	}

	void MotionEstimation::setisGPSEnabled(bool gps) {
		isGPSEnabled_ = gps;
	}

	void MotionEstimation::setisGPSSLAMThresholdEnabled(bool value) {
		isGPSSLAMThresholdEnabled_ = value;
	}

	void MotionEstimation::setisAttitudeStarted(bool Value) {
		isAttitudeStarted_ = Value;
	}

	void MotionEstimation::setisPositionStarted(bool Value) {
		isPositionStarted_ = Value;
	}

	void MotionEstimation::setisMagAvailable(bool Value) {
		isMagAvailable_ = Value;
	}

	void MotionEstimation::setisAccAvailable(bool Value) {
		isAccAvailable_ = Value;
	}

	void MotionEstimation::setisSLAMAvailable(bool orientationValue,bool positionValue){
		if (orientationValue && positionValue) {
			isSLAMAvailable_ = true;
		}
		else {
			isSLAMAvailable_ = false;
		}
		isSLAMOrientationAvailable_ = orientationValue;
		isSLAMPositionAvailable_ = positionValue;
	}
	void MotionEstimation::setisGPSAvailable(bool Value) {
		isGPSAvailable_ = Value;
	}
	void MotionEstimation::setisMotionEstimationStarted(bool Value) {
		isMotionEstimationStarted_ = Value;
	}

	void MotionEstimation::setGyroVar(Vector3axes gyroVarValue, Vector3axes gyroBiasVarValue) {
		varg = gyroVarValue;
		varbg = gyroBiasVarValue;
	}
	void MotionEstimation::setAccVar(Vector3axes accVarValue, Vector3axes accBiasVarValue) {
		vara = accVarValue;
		varba = accBiasVarValue;
	}
	void MotionEstimation::setMagVar(Vector3axes magVarValue) {
		varm = magVarValue;
	}
	void MotionEstimation::setSLAMVar(Vector3axes eSLAMVarValue, Vector3axes pSLAMVarValue) {
		vareSLAM = eSLAMVarValue;
		varpSLAM = pSLAMVarValue;
	}
	void MotionEstimation::setGPSVar(Vector3axes pGPSVarValue) {
		varpGPS = pGPSVarValue;
	}
	void  MotionEstimation::setAccValue(Vector3axes aAccValue) {
		aAcc = aAccValue;
		if (!isMotionEstimationStarted_) {
			apHat = aAcc;
			apHatPrevious = aAcc;
		}
	}
	void  MotionEstimation::setMagValue(Vector3axes mMagValue) {
		mMag = mMagValue;
	}
	void  MotionEstimation::setGyroValue(Vector3axes omegaGyroValue){
		omegaGyro = omegaGyroValue;
	}

	void  MotionEstimation::setGPSValue(Vector3axes GpGPSValue){
		GpGPS = GpGPSValue;
	}
	void  MotionEstimation::setSLAMValue(VectorQuat LGqSLAMValue, Vector3axes GpSLAMValue){
		GpSLAM = GpSLAMValue;
		LGqSLAM = LGqSLAMValue;
	}
	void MotionEstimation::setINSthreshold(double INSthresholdValue) {
		INSthreshold = INSthresholdValue;
	}
	void MotionEstimation::setPosGPSSLAMThreshold(double posGPSSLAMThresholdValue) {
		posGPSSLAMThreshold = posGPSSLAMThresholdValue;
	}

	void  MotionEstimation::MotionEstimation::setSampleTime (double dtValue) {
		dt = dtValue;
	}
	void MotionEstimation::setEarthGravity(Vector3axes geValue){
		ge = geValue;
	}
	void MotionEstimation::setEarthMagneticField(Vector3axes meValue){
		me = meValue;
	}
	void MotionEstimation::setAlphaLPF(double alphaValue){
		alphaLPF = alphaValue;
	}
	void MotionEstimation::setMatrixTaug(Matrix3d TaugValue){
		Lambdag = TaugValue;
	}
	void MotionEstimation::setMatrixLambdaa(Matrix3d TauaValue){
		Lambdaa = TauaValue;
	}
	void MotionEstimation::setButterHPFilter(Vector2d bHPValue, Vector2d aHPValue){
		bHP = bHPValue;
		aHP = aHPValue;
	}
	void MotionEstimation::setButterLPFilter(Vector2d bLPValue, Vector2d aLPValue){
		bLP = bLPValue;
		aLP = aLPValue;
	}
	void MotionEstimation::startAttitude(double roll, double pitch, double yaw) {
		LGqHat = Conversion::angle2quatZYX(yaw,pitch,roll);
		isAttitudeStarted_ = true;
	}
	void MotionEstimation::startPosition(Vector3axes position) {
		GpHat = position;
		GppHat = position;
		GpSLAM = position;
		GpGPS = position;
		isPositionStarted_ = true;
	}
	bool MotionEstimation::isAttitudeStarted(void){
		return isAttitudeStarted_;
	}
	bool MotionEstimation::isPositionStarted(void){
		return isPositionStarted_;
	}
	bool MotionEstimation::isMotionEstimationStarted(void){
		return isMotionEstimationStarted_;
	}

	bool MotionEstimation::isINSEnabled(void) {
		return isINSEnabled_;
	}

	bool MotionEstimation::isGyroEnabled(void) {
		return isGyroEnabled_;
	}
	bool MotionEstimation::isAccelEnabled(void) {
		return isAccelEnabled_;
	}
	bool MotionEstimation::isMagEnabled(void) {
		return isMagEnabled_;
	}
	bool MotionEstimation::isSLAMEnabled(void) {
		return isSLAMEnabled_;
	}
	bool MotionEstimation::isSLAMOrientationEnabled(void) {
		return isSLAMOrientationEnabled_;
	}
	bool MotionEstimation::isSLAMPositionEnabled(void) {
		return isSLAMPositionEnabled_;
	}

	bool MotionEstimation::isSLAMAvailable(void) {
		return isSLAMAvailable_;
	}
	bool MotionEstimation::isSLAMOrientationAvailable(void) {
		return isSLAMOrientationAvailable_;
	}
	bool MotionEstimation::isSLAMPositionAvailable(void) {
		return isSLAMPositionAvailable_;
	}

	bool MotionEstimation::isGPSEnabled(void) {
		return isGPSEnabled_;
	}

	bool MotionEstimation::isGPSSLAMThresholdEnabled(void) {
		return isGPSSLAMThresholdEnabled_;
	}

	void MotionEstimation::propagation(void) {
    	// Gyroscope
    	bgpHat = bgHat;
        omegagpHat = omegaGyro - bgpHat;

        // Accelerometer
        apHatPrevious = apHat;
        bapHat = baHat;
        apHat = aAcc - bapHat;

        // Quaternion propagation
        LGqpHat = quaternionPropagation(omegagpHat, LGqHat,dt);

        // Stationary checking

//        if (isINSEnabled_) {
		apHatMagFilt1Previous = apHatMagFilt1;
		apHatMagFilt1 = filterEval(bHP,aHP,apHat.norm(),apHatPrevious.norm(),apHatMagFilt1);
		apHatMagFilt2 = filterEval(bLP,aLP,abs(apHatMagFilt1),abs(apHatMagFilt1Previous),apHatMagFilt2);
		isStationary_ = apHatMagFilt2 < INSthreshold;
        // INS propagation
		GvpHat = insPropagation(LGqHat,apHat, GvHat,ge, dt, isStationary_);
//        }
//        else {
//		GvpHat = insPropagation(LGqHat,apHat, GvHat,ge, dt);
//        }
		GppHat = insPropagation(GvpHat, GpHat, dt);
	}

	VectorObser MotionEstimation::observationError(void) {
		VectorObser obsError;
		obsError = obsError.Zero();

		Matrix3d LGrotpHat = quat2dcmZYX(LGqpHat);

		stateMarkov = stateMarkov.Zero();

		if (isSLAMOrientationAvailable_) {
			VectorQuat LGqError;
			LGqError = Conversion::quatmultiply(LGqSLAM,Conversion::quatconj(LGqpHat));
//			LGqError = Conversion::quatmultiply(LGqHat,Conversion::quatconj(LGqSLAM));
//			obsError.segment(0,3) = LGqError.segment(1,3);

			Matrix3d LGrotSLAM = quat2dcmZYX(LGqSLAM);
			obsError.segment(0,3) = LGrotSLAM*me - LGrotpHat*me;;

//			obsError.segment(0,3) = Vector3axes::Zero();
//			obsError(2) = LGqError(3);
//			cout << "SLAM Orientation Observation" << endl;
//			cout << "obsError.segment(0,3) = " << obsError.segment(0,3).transpose() << endl;
			stateMarkov(0) = 1;
		}
		else if (isMagAvailable_) {
			obsError.segment(0,3) = mMag- LGrotpHat*me;
			stateMarkov(0) = 2;
		}
		if (isAccAvailable_) {
//			obsError.segment(3,3) = (aAcc - bapHat) - (-LGrotpHat*ge);
			obsError.segment(3,3) = (aAcc) - (-LGrotpHat*ge);
			stateMarkov(1) = 1;
		}
		if (isSLAMPositionAvailable_) {
			obsError.segment(6,3) = GpSLAM - GppHat;
//			cout << "SLAM Position Observation" << endl;
			stateMarkov(2) = 1;
		}
		else if (isGPSAvailable_) {
			obsError.segment(6,3) = GpGPS - GppHat;
//			cout << "GPS Position Observation" << endl;
			stateMarkov(2) = 2;
		}

//		if (isMagAvailable_) {
//			obsError.segment(0,3) = mMag- LGrotpHat*me;
//		}
//		if (isAccAvailable_ ) { // && isStationary_) {
//			obsError.segment(3,3) = (aAcc - bapHat) - (-LGrotpHat*ge);
//		}
//		if (isSLAMOrientationAvailable_) {
//			VectorQuat LGqError;
//			LGqError = Conversion::quatmultiply(LGqSLAM,Conversion::quatconj(LGqHat));
//			obsError.segment(6,3) = LGqError.segment(1,3);
////			obsError.segment(6,3) = Vector3axes::Zero();
////			obsError(8) = LGqError(3);
////			cout << "SLAM Orientation Observation" << endl;
//		}
//		if (isSLAMPositionAvailable_) {
//			obsError.segment(9,3) = GpSLAM - GppHat;
////			cout << "SLAM Position Observation" << endl;
//		}
//		if(isGPSAvailable_) {
//			obsError.segment(12,3) = GpGPS - GppHat;
//		}

		return obsError;
	}

	void MotionEstimation::state2variables(VectorState x) {
		// Orientation computation
		Vector3axes deltaAngle = x.segment(0,3);
		LGqHat = deltaAngle2quat(deltaAngle, LGqpHat);

		// TODO - The angles computation is not necessary for the system, only using to compare
		anglesHat = quat2angleZYX(LGqHat);

		// Checking the orientation quaternion norm
		if (LGqHat.norm() > 1.1) {
			ROS_WARN("The norm of the orientation quaternion LGqHat is : %f.", LGqHat.norm());
		}
		// Gyro bias computation
		Vector3axes deltabgHat = x.segment(3,3);
		bgHat = bgHat + deltabgHat;
		omegagHat = omegaGyro - bgHat;

		// Accelerometer bias computation
		Vector3axes deltabaHat = x.segment(9,3);
		baHat = baHat + deltabaHat;
		aHat = aAcc - baHat;

		// Position computation
		if ((isGPSAvailable_ | isSLAMPositionAvailable_) && isPositionStarted_) {
			// Position computation
			Vector3axes deltaGpHat = x.segment(12,3);
			GpHat = deltaGpHat + GpHat;
			// Velocity computation
			Vector3axes deltaGvHat = x.segment(6,3);
			GvHat = deltaGvHat + GvHat;
//			cout << "Position computation" << endl;
		}
//		else {
//		else if (isPositionStarted_ ){
		else if (isPositionStarted_ | isINSEnabled_){
//		if (isINSEnabled_){
			// Using INS propagation
			GpHat = GppHat;
			GvHat = GvpHat;
//			cout << "INS propagation" << endl;
		}

        // Angular velocity computation
		Matrix3d LGrotpHat = quat2dcmZYX(LGqpHat);
        danglesHat = LGrotpHat.transpose()*omegagpHat;

        // TODO - Using only to compate the angles, not necessary for the system
        anglesGyro = danglesHat*dt + anglesGyro;
	}

	MatrixF MotionEstimation::montionSys15Matrices(VectorQuat LGq, Vector3axes omega, Vector3axes La, const float dt) {
		MatrixF A;
		MatrixF F;

		MatrixF Istate;

		A = A.Zero();
	    Istate = Istate.Identity();
		Matrix3d minusRT = -quat2dcmZYX(LGq).transpose();
		Matrix3d OMEGAcross = crossMatrix(omega);
		Matrix3d ACCELcross = crossMatrix(La);
		Matrix3d I3 = Matrix3d::Identity();
		A.block(0,0,3,3) = -OMEGAcross;
		A.block(0,3,3,3) = -I3;
		A.block(3,3,3,3) = -Lambdag;
		A.block(6,0,3,3) = minusRT*ACCELcross;
		A.block(6,9,3,3) = minusRT;
		A.block(9,9,3,3) = -Lambdaa;
		A.block(12,6,3,3) = I3;

		F = Istate+A*dt;

		return F;
	}

	MatrixG MotionEstimation::montionSys15Matrices(VectorQuat LGq, const float dt) {
		MatrixG B;
		MatrixG G;

		B = B.Zero();

		Matrix3d minusRT = -quat2dcmZYX(LGq).transpose();
		Matrix3d I3 = Matrix3d::Identity();

		B.block(0,0,3,3) = -I3;
		B.block(3,3,3,3) =  I3;
	    B.block(6,6,3,3) = minusRT;
	    B.block(9,9,3,3) = I3;
		G = sqrt(dt)*B;

		return G;
	}

	MatrixH MotionEstimation::montionSys15Matrices(VectorQuat LGq, Vector3axes Gme, Vector3axes Gge) {
		MatrixH H;
		H = H.Zero();
		Matrix3d R = quat2dcmZYX(LGq);

		Matrix3d GMEcross = crossMatrix(Gme);
		Matrix3d GGEcross = crossMatrix(Gge);
		Matrix3d I3 = I3.Identity();
//		Matrix3d Zero3 = Zero3.Zero();

		if (isSLAMOrientationAvailable_) {
//			H.block(0,0,3,3) = I3; 	// SLAM and Estimated Angle error
			H.block(0,0,3,3) = R*GMEcross; 	// Magnetometer
		}
		else if (isMagAvailable_) {
			H.block(0,0,3,3) = R*GMEcross; 	// Magnetometer
		}
//		else {
//			H.block(0,0,3,3) = Zero3; 	// None
//		}

		if (isAccAvailable_) {
			H.block(3,0,3,3) = -R*GGEcross; // Accelerometer
			H.block(3,9,3,3) = I3; // Accelerometer
		}
//		else {
//			H.block(3,0,3,3) = Zero3; // None
//		}

		if (isSLAMPositionAvailable_) {
			H.block(6,12,3,3) = I3;         // SLAM Position
		}
		else if (isGPSAvailable_) {
			H.block(6,12,3,3) = I3;         // GPS Position
		}
//		else {
//			H.block(6,12,3,3) = Zero3;     // None
//		}

//		H.block(0,0,3,3) = R*GMEcross; 	// Magnetometer
//		H.block(3,0,3,3) = -R*GGEcross; // Accelerometer
//		H.block(6,0,3,3) = I3;          // SLAM and Estimated Angle error
//	    H.block(9,12,3,3) = I3;         // SLAM Position
//	    H.block(12,12,3,3) = I3;         // GPS Position

		return H;
	}



} // namespace ME
