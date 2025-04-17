/*
 * scale_estimation.h
 *
 *  Created on: 1 Jul 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef SCALE_ESTIMATION_H_
#define SCALE_ESTIMATION_H_

#include "me_definitions.h"
#include "sensors.h"
#include "conversion.h"
#include <iostream>
#include <fstream>


using namespace ME::Conversion;

namespace ME {

	class ScaleEstimation {
		double dt;
		Vector3axes GpSLAM;
		Vector3axes GpSLAMPrevious;
		Vector3axes dGpSLAM;
		Vector3axes deltaSLAM;
		Vector3axes dispSLAM;
		VectorQuat LGqRef;
		Vector3axes LaRefPrevious;
		Vector3axes LaRef;
		Vector3axes ge;
		Vector3axes GvRef;
		Vector3axes GpRef;
		Vector3axes GpRefPrevious;
		Vector3axes dGpRef;
		Vector3axes deltaRef;
		Vector3axes dispRef;

		double velSLAM;
		double velSLAMThreshold;

		double LaRefMagFilt1Previous;
		double LaRefMagFilt1;
		double LaRefMagFilt2;
		Vector2d bHP, aHP, bLP, aLP;
		double INSthreshold;

		double alphaLPF;
		bool isScaledEstimationSLAMStarted_;
		bool isScaledEstimationRefStarted_;
		bool isVelSLAMThresholdSet_;
		bool stationarySLAM_;

		bool stationaryAcc_;

		bool stationarySLAMPrevious_;
		bool stationarySLAMStart_;
		bool stationarySLAMEnd_;
		bool dispStart_;
		int dispCount;
		int dispCountSize;
		int scaleCount;
		double scale;
		double scaleLoop;

		double scaleLoopX;
		double scaleLoopY;
		double scaleLoopZ;

		double scalePropThreshold;

		bool isScaleProportional_;

		double scaleLoopFilt;
		int scaleStorageNumber;
		int scaleOption;
		bool isScaleAvailable_;
		bool isScaleLoopStarted_;

		bool autoVelSLAMThreshold_;

	public:
		ScaleEstimation ();
		bool isScaleAvailable(void);
		void setisScaleAvailable(bool Value);
		void setAutoVelSLAMThreshold(bool Value);
		void setScalePropThreshold(double scalePropThresholdValue);
		void setScaleStorageNumber(int scaleStorageNumberValue);
		void setDispCountSize(int dispCountSizeValue);
		void setSLAMPosition(Vector3axes GpSLAMValue);
		void setRefPosition(Vector3axes GpRefValue);
		void setVelMagSLAM(double velMagSLAMValue);
		void setRefValues(VectorQuat LGqRefValue, Vector3axes LaRefValue, Vector3axes geValue, double dtValue);

		void setAlphaLPF(double alphaValue);
		void setButterHPFilter(Vector2d bHPValue, Vector2d aHPValue);
		void setButterLPFilter(Vector2d bLPValue, Vector2d aLPValue);

		void setSampleTime (double dtValue);
		void setINSthreshold(double INSthresholdValue);
		void setVelSLAMThreshold(double  velSLAMThresholdValue);
		double getErrorNorm(void);
		void reset(void);
		void setScaleOption(int optionValue);
		bool getAutoVelSLAMThreshold(void);
		int getScaleOption(void);
		double getScale(void);
		double getVelSLAMFiltered(void);
		double getVelSLAMThreshold(void);
		Vector3axes getRefPosition(void);
		void propagation(void);
		void scaleEstimation(void);
	};

} // namespace ME


#endif // SCALE_ESTIMATION_H
