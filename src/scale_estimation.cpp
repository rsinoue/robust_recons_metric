/*
 * scale_estimation.cpp
 *
 *  Created on: 1 Jul 2016
 *      Author: Roberto Santos Inoue
 */


#include "scale_estimation.h"

namespace ME {

	ScaleEstimation::ScaleEstimation(){
		dt = 0.01;
		GpSLAM = GpSLAM.Zero();
		GpSLAMPrevious = GpSLAMPrevious.Zero();
		dGpSLAM = dGpSLAM.Zero();
		deltaSLAM = deltaSLAM.Zero();
		dispSLAM = dispSLAM.Zero();


		LGqRef = LGqRef.Zero();

		LaRef = LaRef.Zero();

		ge = ge.Zero();

		GvRef = GvRef.Zero();

		GpRef = GpRef.Zero();
		GpRefPrevious = GpRefPrevious.Zero();
		dGpRef = dGpRef.Zero();
		deltaRef = dGpRef.Zero();
		dispRef = dispRef.Zero();
		velSLAM = 0;
		velSLAMThreshold = 0.06;

		LaRefMagFilt1Previous = 0;
		LaRefMagFilt1 = 0;
		LaRefMagFilt2 = 0;

		bHP << 0.999968585060383,  -0.999968585060383; // = bHP.Zero();
		aHP << 1.000000000000000,  -0.999937170120766; // = aHP.Zero();
		bLP << 0.136728735997319,   0.136728735997319; // = bLP.Zero();
		aLP << 1.000000000000000,  -0.726542528005361; // = aLP.Zero();

	    INSthreshold = 0.01;

		alphaLPF = 0.9;
		isScaledEstimationSLAMStarted_ = false;
		isVelSLAMThresholdSet_ = false;
		stationarySLAM_ = true;
		stationaryAcc_ = true;
		stationarySLAMPrevious_ = true;
		stationarySLAMStart_ = false;
		stationarySLAMEnd_ = false;
		dispStart_ = false;

		dispCount = 0;
		dispCountSize = 500;

		scaleCount = 0;

		scale = 1;
		scaleLoop = 1;


		scaleLoopX = 1;
		scaleLoopY = 1;
		scaleLoopZ = 1;

		scalePropThreshold = 0.3;

		isScaleProportional_ = false;

		scaleLoopFilt = 1;
		scaleStorageNumber = 3;

		scaleOption = 0;

		isScaleAvailable_= false;
		isScaleLoopStarted_ = false;

		autoVelSLAMThreshold_ = false;
	}

	bool ScaleEstimation::isScaleAvailable(void){
		return isScaleAvailable_;
	}
	void ScaleEstimation::setisScaleAvailable(bool Value) {
		isScaleAvailable_ = Value;
	}
	void ScaleEstimation::setAutoVelSLAMThreshold(bool Value) {
		autoVelSLAMThreshold_ = Value;
	}
	void ScaleEstimation::setScalePropThreshold(double scalePropThresholdValue) {
		scalePropThreshold = scalePropThresholdValue;
	}
	void ScaleEstimation::setScaleStorageNumber(int scaleStorageNumberValue) {
		scaleStorageNumber = scaleStorageNumberValue;
	}

	void ScaleEstimation::setDispCountSize(int dispCountSizeValue) {
		dispCountSize = dispCountSizeValue;
	}

	void ScaleEstimation::setSLAMPosition(Vector3axes GpSLAMValue){
		if (!isScaledEstimationSLAMStarted_) {
			GpSLAM = GpSLAMValue;
			GpSLAMPrevious = GpSLAM;
			isScaledEstimationSLAMStarted_ = true;
		}
		else {
			GpSLAMPrevious = GpSLAM;
			GpSLAM = GpSLAMValue;
		}
	}

	void ScaleEstimation::setRefPosition(Vector3axes GpRefValue) {
		if (!isScaledEstimationRefStarted_) {
			GpRef = GpRefValue;
			GpRefPrevious = GpRef;
			isScaledEstimationRefStarted_ = true;
		}
		else {
			GpRefPrevious = GpRef;
			GpRef = GpRefValue;
		}
	}

	void ScaleEstimation::setVelMagSLAM(double velMagSLAMValue){
		velSLAM = velMagSLAMValue;
	}

	void ScaleEstimation::setRefValues(VectorQuat LGqRefValue, Vector3axes LaRefValue, Vector3axes geValue, double dtValue) {
		LGqRef = LGqRefValue;
		ge = geValue;
		dt = dtValue;
		if (!isScaledEstimationRefStarted_) {
			LaRef = LaRefValue;
			LaRefPrevious = LaRef;
			isScaledEstimationRefStarted_ = true;
		}
		else {
			LaRefPrevious = LaRef;
			LaRef = LaRefValue;
		}
	}
	void ScaleEstimation::setAlphaLPF(double alphaValue){
		alphaLPF = alphaValue;
	}
	void ScaleEstimation::setButterHPFilter(Vector2d bHPValue, Vector2d aHPValue){
		bHP = bHPValue;
		aHP = aHPValue;
	}
	void ScaleEstimation::setButterLPFilter(Vector2d bLPValue, Vector2d aLPValue){
		bLP = bLPValue;
		aLP = aLPValue;
	}
	void ScaleEstimation::setSampleTime (double dtValue) {
		dt = dtValue;
	}
	void ScaleEstimation::setINSthreshold(double INSthresholdValue) {
		INSthreshold = INSthresholdValue;
	}
	void ScaleEstimation::setVelSLAMThreshold(double  velSLAMThresholdValue){
		 velSLAMThreshold =  velSLAMThresholdValue;
	}
	void ScaleEstimation::setScaleOption(int optionValue){
		scaleOption = optionValue;
	}

	bool ScaleEstimation::getAutoVelSLAMThreshold(void) {
		return autoVelSLAMThreshold_;
	}

	int ScaleEstimation::getScaleOption(void){
		return scaleOption;
	}
	double ScaleEstimation::getScale(void) {
		return scale;
	}

	double ScaleEstimation::getVelSLAMFiltered(void) {
		return velSLAM;
	}

	double ScaleEstimation::getVelSLAMThreshold(void) {
		return velSLAMThreshold;
	}

	double ScaleEstimation::getErrorNorm(void) {
		return (GpRef - GpSLAM).norm();
	}

	Vector3axes ScaleEstimation::getRefPosition(void) {
		return GpRef;
	}

	void ScaleEstimation::reset(void) {
		scaleCount = 0;
		isScaleLoopStarted_ = false;
		isScaleAvailable_ = false;
	}

	void ScaleEstimation::propagation(void) {
        // Stationary Accelerometer checking
		LaRefMagFilt1Previous = LaRefMagFilt1;
		LaRefMagFilt1 = filterEval(bHP,aHP,LaRef.norm(),LaRefPrevious.norm(),LaRefMagFilt1);
		LaRefMagFilt2 = filterEval(bLP,aLP,abs(LaRefMagFilt1),abs(LaRefMagFilt1Previous),LaRefMagFilt2);
		stationaryAcc_ = LaRefMagFilt2 < INSthreshold;

		if (stationaryAcc_) {
//			cout << "Stationary Acc" << endl;
		}
		else
		{
//			cout << "In motion Acc" << endl;
		}

        // INS propagation
		GvRef = insPropagation(LGqRef,LaRef,GvRef,ge,dt,stationaryAcc_ && stationarySLAM_);
		GpRefPrevious = GpRef;
		GpRef = insPropagation(GvRef, GpRef, dt);
	}

	void ScaleEstimation::scaleEstimation(void){
		if (scaleOption == 3) {
			propagation();
		}
		if (isScaledEstimationSLAMStarted_ && isScaledEstimationRefStarted_) {
			stationarySLAMPrevious_ = stationarySLAM_;
			stationarySLAM_ = velSLAM < velSLAMThreshold;

			if (stationarySLAM_) {
//				cout << "Stationary SLAM" << endl;
			}
			else
			{
//				cout << "In motion SLAM" << endl;
			}
			stationarySLAMStart_ = (stationarySLAM_ - stationarySLAMPrevious_) == 1;
			stationarySLAMEnd_ = (stationarySLAM_ - stationarySLAMPrevious_) == -1;

			if (stationarySLAMStart_) {
				cout << "Stationary  SLAM Started" << endl;
			}
			if (stationarySLAMEnd_) {
				cout << "Stationary  SLAM Ended" << endl;
			}
			if (stationarySLAMEnd_) {
				dispCount = 0;
				dispStart_ = true;
				dispSLAM = dispSLAM.Zero();
				dispRef = dispRef.Zero();
			}
			if (stationarySLAMStart_) {
				dispStart_ = false;
			}
			if (stationarySLAMStart_ && (dispCount > dispCountSize)) {
				cout << "dispCount = [" << dispCount << "];" << endl;
				cout << "scaleCount = [" << scaleCount << "];" << endl;
				cout << "dispRef.norm() = [" << dispRef.norm() << "];" << endl;
				cout << "dispSLAM.norm() = [" << dispSLAM.norm() << "];" << endl;
				cout << "dispRef.norm()/dispSLAM.norm() = [" << dispRef.norm()/dispSLAM.norm() << "];" << endl;

				scaleLoopX = dispRef(0)/dispSLAM(0);
				scaleLoopY = dispRef(1)/dispSLAM(1);
				scaleLoopZ = dispRef(2)/dispSLAM(2);
				cout << "[scaleLoopX,scaleLoopX,scaleLoopX] = [" << scaleLoopX << " " << scaleLoopY << " " << scaleLoopZ << "]" << endl;

				isScaleProportional_ = abs(scaleLoopX - scaleLoopY)/(sqrt(scaleLoopX*scaleLoopX + scaleLoopY*scaleLoopY)) < scalePropThreshold;

//				scaleLoop = scaleLoopX; //dispRef.segment(0,2).norm()/dispSLAM.segment(0,2).norm();

				scaleLoop = dispRef.segment(0,2).norm()/dispSLAM.segment(0,2).norm();

				cout << "scaleLoop = [" << scaleLoop << ";" << endl;

//				if (isScaleProportional_ && scaleLoop > 0) {
				if (scaleLoop > 0) {

					// Initial condition
					if (!isScaleLoopStarted_) {
						scaleLoopFilt = scaleLoop;
						isScaleLoopStarted_ = true;
					}
					cout << "scaleLoopFiltPrevious = [" << scaleLoopFilt << ";" << endl;
					scaleLoopFilt = lowPassFilter(scaleLoop,scaleLoopFilt,alphaLPF);
					cout << "scaleLoopFilt = [" << scaleLoopFilt << ";" << endl;
					scaleCount = scaleCount + 1;
				}

				if (scaleCount == scaleStorageNumber) {
//					scale = scaleStorage.mean();
					scale = scaleLoopFilt;
					isScaleAvailable_ = true;
					cout << "scale = [" << scale << "];" << endl;
//					scaleCount = 0;
				}
//				else if (scaleCount > scaleStorageNumber) {
//					double scalePropor = abs(scale/scaleLoopFilt - 1);
//					if (scalePropor > 0.2){
//						scale = scaleLoopFilt;
//						cout << "scale = [" << scale << "];" << endl;
//					}
//				}
			}
			if (dispStart_) {
				dispCount++;
				deltaSLAM = GpSLAM - GpSLAMPrevious;
				deltaRef = GpRef - GpRefPrevious;

//				dispSLAM = deltaSLAM.cwiseAbs() + dispSLAM;
//				dispRef = deltaRef.cwiseAbs() + dispRef;

				dispSLAM = deltaSLAM + dispSLAM;
				dispRef = deltaRef + dispRef;
			}
		}
	}
} // namespace ME

