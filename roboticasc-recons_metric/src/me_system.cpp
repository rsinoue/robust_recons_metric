/*
 * me_system.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "me_system.h"

namespace ME {

	System::System() {
		contPosNumberIter = 100;
		contPos = 0; // Position Start
		contVelMagSLAM = 0;

		logPath = "./";
	    loadSettings(n);
	    loadTopics(n);
	    openLogsFiles(logPath);

	    tIMU = 0;
	    tIMUPrevious = 0;
	}

	System::~System () {
		closeLogsFiles();
	}

	void System::openLogsFiles(const string &logPath) {
	    // CSV log files

//		string mePath = logPath + "melog.csv";
//		mefile.open(mePath.c_str());
//
//		string mebiasPath = logPath + "mebiaslog.csv";
//		mebiasfile.open(mebiasPath.c_str());
//
//		string gpsPath = logPath + "gpslog.csv";
//		gpsfile.open(gpsPath.c_str());
//
//		string imuPath = logPath + "imulog.csv";
//		imufile.open(imuPath.c_str());
//
//		string gyroPath = logPath + "gyrolog.csv";
//		gyrofile.open(gyroPath.c_str());
//
//		string accmagPath = logPath + "accmaglog.csv";
//		accmagfile.open(accmagPath.c_str());
//
//		string accPath = logPath + "acclog.csv";
//		accfile.open(accPath.c_str());

		string slamPath = logPath + "slamlog.csv";
		slamfile.open(slamPath.c_str());

		string resultPath = logPath + "resultslog.csv";
		resultsfile.open(resultPath.c_str());
	}
	void System::closeLogsFiles(void) {
	  	// Close CSV log file
//	  	mefile.close();
//	  	mebiasfile.close();
//	  	gpsfile.close();
//	  	imufile.close();
//	  	gyrofile.close();
//	  	accmagfile.close();
	  	slamfile.close();
	  	resultsfile.close();
	}
	void System::loadTopics(ros::NodeHandle &n) {
	    imu_subscriber = n.subscribe("/imu/data", 1, &ME::System::imuCallback,this);
	    mag_subscriber = n.subscribe("/imu/mag", 1, &ME::System::magCallback,this);
	    gps_subscriber = n.subscribe("/gps/fix", 1, &ME::System::gpsFixCallback,this);
//	    gps_subscriber = n.subscribe("/gps/extended_fix", 1, &ME::System::gpsExtFixCallback,this);
//	    gpsRef_subscriber = n.subscribe("/ref/fix", 1, &ME::System::gpsFixCallback,this);
	    gpsRef_subscriber = n.subscribe("/ref/extended_fix", 1, &ME::System::gpsRefExtFixCallback,this);
	    //slam_subscriber = n.subscribe("/orb_slam2/pose", 1,&ME::System::slamCallback,this);
//	    slam_subscriber = n.subscribe("/lsd_slam/pose", 1,&ME::System::slamCallback,this);
	    slam_subscriber = n.subscribe("/slam/pose", 1,&ME::System::slamCallback,this);

	    odometry_publisher = n.advertise<nav_msgs::Odometry>("/me/odometry",1);
	    biasAcc_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/me/biasAcc",1);
	    biasGyro_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/me/biasGyro",1);
	    scaleSLAM_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/me/scaleSLAM",1);
//	    positionGPS_publisher = n.advertise<geometry_msgs::Point>("recons_metric/positionGPS",1);
//	    anglesIMU_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/anglesIMU",1);
	    anglesEst_publisher = n.advertise<geometry_msgs::Vector3Stamped>("/me/anglesEst",1);
//	    anglesGyro_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/anglesGyro",1);
//	    anglesAccMag_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/anglesAccMag",1);
//	    anglesSLAM_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/anglesSLAM",1);
//	    positionSLAM_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/positionSLAM",1);
	}


	void System::loadSettings(ros::NodeHandle &n) {

		double alphaLPFGyro;
		if (n.getParam("/recons_metric/alphaLPFGyro",alphaLPFGyro)) {
			gyro.setAlphaLPF(alphaLPFGyro);
		}
		double alphaLPFAcc;
		if (n.getParam("/recons_metric/alphaLPFAcc",alphaLPFAcc)) {
			acc.setAlphaLPF(alphaLPFAcc);
		}
		double alphaLPFMag;
		if (n.getParam("/recons_metric/alphaLPFMag",alphaLPFMag)) {
			mag.setAlphaLPF(alphaLPFMag);
		}
		double alphaLPFme;
		if (n.getParam("/recons_metric/alphaLPFme",alphaLPFme)) {
			me.setAlphaLPF(alphaLPFme);
		}
		double alphaLPFse;
		if (n.getParam("/recons_metric/alphaLPFse",alphaLPFse)) {
			se.setAlphaLPF(alphaLPFse);
		}
		double magDec;
		if (n.getParam("/recons_metric/magDec",magDec)) {
			gps.setMagDec(magDec);
			gpsRef.setMagDec(magDec);
		}
		double accBounds;
		if (n.getParam("/recons_metric/boundsPercAcc",accBounds)) {
			acc.setBoundsPerc(accBounds);
		}
		double magBounds;
		if (n.getParam("/recons_metric/boundsPercMag",magBounds)) {
			mag.setBoundsPerc(magBounds);
		}
		double INSthreshold;
		if (n.getParam("/recons_metric/INSthreshold",INSthreshold)) {
			me.setINSthreshold(INSthreshold);
			se.setINSthreshold(INSthreshold);
		}
		double dt;
		if (n.getParam("/recons_metric/dt",dt)) {
			me.setSampleTime(dt);
			se.setSampleTime(dt);
		}
		bool isVargObtained = false;
		vector<double> varg;
		if (n.getParam("/recons_metric/varg",varg)) {
			isVargObtained = true;
		}
		bool isVarbgObtained = false;
		vector<double> varbg;
		if (n.getParam("/recons_metric/varbg",varbg)) {
			isVarbgObtained = true;
		}
		vector<double> lambdagValues;
		if (n.getParam("/recons_metric/lambdag",lambdagValues)){
			Matrix3d Lambdag = Matrix3d::Zero();
			Lambdag.diagonal() = VectorXd::Map(&lambdagValues[0],size3axes);
			me.setMatrixTaug(Lambdag);
//			cout << "Lambdag = [" << Lambdag << "]" <<  endl;
		}
		bool isVaraObtained = false;
		vector<double> vara;
		if (n.getParam("/recons_metric/vara",vara)) {
			isVaraObtained = true;
		}
		bool isVarbaObtained = false;
		vector<double> varba;
		if (n.getParam("/recons_metric/varba",varba)) {
			isVarbaObtained = true;
		}
		vector<double> lambdaaValues;
		if (n.getParam("/recons_metric/lambdaa",lambdaaValues)){
			Matrix3d Lambdaa = Matrix3d::Zero();
			Lambdaa.diagonal() = VectorXd::Map(&lambdaaValues[0],size3axes);
			me.setMatrixLambdaa(Lambdaa);
//			cout << "Lambdaa = [" << Lambdaa << "]" <<  endl;
		}
		bool isVarmObtained = false;
		vector<double> varm;
		if (n.getParam("/recons_metric/varm",varm)) {
			isVarmObtained = true;
		}
		bool isVarpGPSObtained = false;
		vector<double> varpGPS;
		if (n.getParam("/recons_metric/varpGPS",varpGPS)) {
			isVarpGPSObtained = true;
		}
		bool isVareSLAMObtained = false;
		vector<double> vareSLAM;
		if (n.getParam("/recons_metric/vareSLAM",vareSLAM)) {
			isVareSLAMObtained = true;
		}
		bool isVarpSLAMObtained = false;
		vector<double> varpSLAM;
		if (n.getParam("/recons_metric/varpSLAM",varpSLAM)) {
			isVarpSLAMObtained = true;
		}
		vector<double> Pdiagdata;
		if (n.getParam("/recons_metric/Pdiag/data",Pdiagdata)){
			MatrixP P = MatrixP::Zero();

			P.diagonal() = VectorXd::Map(&Pdiagdata[0],sizeState);
			kf.setMatrixP(P);
		}

		int autoGravEst;
		if (n.getParam("/recons_metric/autoGravEst",autoGravEst)) {
			if (autoGravEst == 1) {
				acc.setAutoGravEst(true);
				cout << "setAutoGravEst = true" <<  endl;
			}
			if (autoGravEst == 0) {
				acc.setAutoGravEst(false);
				cout << "setAutoGravEst = false" <<  endl;
			}
		}

		vector<double> geValues;
		if (n.getParam("/recons_metric/ge",geValues)){
			Vector3axes ge = Vector3axes::Zero();
			ge = Vector3axes::Map(&geValues[0],size3axes);
			acc.setGlobalValue(ge);
		}

		int autoMagFieldEst;
		if (n.getParam("/recons_metric/autoGravEst",autoMagFieldEst)) {
			if (autoMagFieldEst == 1) {
				mag.setAutoMagFieldEst(true);
				cout << "autoMagFieldEst = true" <<  endl;
			}
			if (autoMagFieldEst == 0) {
				mag.setAutoMagFieldEst(false);
				cout << "autoMagFieldEst = false" <<  endl;
			}
		}
		vector<double> meValues;
		if (n.getParam("/recons_metric/me",meValues)){
			Vector3axes me = Vector3axes::Zero();
			me = Vector3axes::Map(&meValues[0],size3axes);
			mag.setGlobalValue(me);
		}

		bool isbHPObtained = false;
		vector<double> bHPValues;
		if (n.getParam("/recons_metric/bHP",bHPValues)){
			isbHPObtained = true;
		}
		bool isaHPObtained = false;
		vector<double> aHPValues;
		if (n.getParam("/recons_metric/bHP",aHPValues)){
			isaHPObtained = true;
		}
		if (isbHPObtained && isaHPObtained) {
			Vector2d bHP = Vector2d::Zero();
			bHP = Vector2d::Map(&bHPValues[0],2);

			Vector2d aHP = Vector2d::Zero();
			aHP = Vector2d::Map(&aHPValues[0],2);

			me.setButterHPFilter(bHP,aHP);
			se.setButterHPFilter(bHP,aHP);
		}

		bool isbLPObtained = false;
		vector<double> bLPValues;
		if (n.getParam("/recons_metric/bLP",bLPValues)){
			isbLPObtained = true;
		}
		bool isaLPObtained = false;
		vector<double> aLPValues;
		if (n.getParam("/recons_metric/bHP",aLPValues)){
			bool isaLPObtained = true;
		}
		if (isbLPObtained && isaLPObtained) {
			Vector2d bLP = Vector2d::Zero();
			bLP = Vector2d::Map(&bLPValues[0],2);

			Vector2d aLP = Vector2d::Zero();
			aLP = Vector2d::Map(&aLPValues[0],2);
			me.setButterLPFilter(bLP,aLP);
			se.setButterLPFilter(bLP,aLP);
		}


		int yawOption;
		if (n.getParam("/recons_metric/yawOption",yawOption)) {
			// 0: Initial condition equal zero
			// 1: Initial condition given by the magnetometer
			// 2: Initial condition given by the GPS // Not implemented
			me.setYawOption(yawOption);

			switch(yawOption) {
				case 0: cout << "yawOption = " << yawOption << ". "  << "Initial condition equal zero."<< endl;
				case 1: cout << "yawOption = " << yawOption << ". "  << "Initial condition given by the magnetometer."<< endl;
//				case 2: cout << "Initial condition given by the GPS."<< endl;
			}
		}

		// Rotation Matrix CGR and Displacement GCd
		bool isCGqObtained = false;
		vector<double> CGqValue;
		if (n.getParam("/recons_metric/CGq",CGqValue)){
			isCGqObtained = true;
		}

		bool isGCdObtained = false;
		vector<double> GCdValue;
		if (n.getParam("/recons_metric/GCd",GCdValue)){
			isGCdObtained = true;
		}

		if (isCGqObtained && isGCdObtained ) {
			VectorQuat CGq = VectorQuat::Zero();
			CGq = VectorQuat::Map(&CGqValue[0],sizeQuat);
			Vector3axes GCd = Vector3axes::Zero();
			GCd = Vector3axes::Map(&GCdValue[0],size3axes);
			slam.setCamera2GlobalTransformation(CGq,GCd);
			cout << "Setting SLAM CGq = [" << CGq.transpose() <<"];"<< endl;
			cout << "Setting SLAM GCd = [" << GCd.transpose() <<"];"<< endl;
		}

		bool autoVelSLAMThreshold;
		if (n.getParam("/recons_metric/autoVelSLAMThreshold",autoVelSLAMThreshold)) {
			if (autoVelSLAMThreshold == true) {
				se.setAutoVelSLAMThreshold(true);
				cout << "autoVelSLAMThreshold = true" <<  endl;
			}
			if (autoVelSLAMThreshold == 0) {
				se.setAutoVelSLAMThreshold(false);
				cout << "autoVelSLAMThreshold = false" <<  endl;
			}
		}

		double velSLAMThreshold;
		if (n.getParam("/recons_metric/velSLAMThreshold",velSLAMThreshold)) {
			se.setVelSLAMThreshold(velSLAMThreshold);
		}

		double scaleSLAM;
		if (n.getParam("/recons_metric/scaleSLAM",scaleSLAM)) {
			slam.setScaleSLAM(scaleSLAM);
		}


		int scaleStorageNumber;
		if (n.getParam("/recons_metric/scaleStorageNumber",scaleStorageNumber)) {
			se.setScaleStorageNumber(scaleStorageNumber);
		}

		double scaleProporThreshold;
		if (n.getParam("/recons_metric/scaleProporThreshold",scaleProporThreshold)) {
			se.setScalePropThreshold(scaleProporThreshold);
		}

		double slamDispCountSize;
		if (n.getParam("/recons_metric/slamDispCountSize",slamDispCountSize)) {
			se.setDispCountSize(slamDispCountSize);
		}

		double errorPosThreshold;
		if (n.getParam("/recons_metric/errorPosThreshold",errorPosThreshold)) {
			me.setPosGPSSLAMThreshold(errorPosThreshold);
			cout << "PosGPSSLAMThreshol = "<< me.getPosGPSSLAMThreshold() << endl;
		}

		int scaleSLAMOption;
		if (n.getParam("/recons_metric/scaleSLAMOption",scaleSLAMOption)) {
			// SLAM scale
			// 0: No scale
			// 1: Fix scale
			// 2: GPS scale estimation
			// 3: Accelerometer scale estimation
			se.setScaleOption(scaleSLAMOption);
			cout << "scaleSLAMOption = [" << scaleSLAMOption <<"];"<< endl;
		}

		bool isINSEnabled;
		if (n.getParam("/recons_metric/isINSEnabled",isINSEnabled)) {
			me.setisINSEnabled(isINSEnabled);
			if (isINSEnabled) {
				cout << "INS is enabled." << endl;
			}
			else {
				cout << "INS is disabled." << endl;
			}
		}

		bool isGyroEnabled;
		if (n.getParam("/recons_metric/isGyroEnabled",isGyroEnabled)) {
			me.setisGyroEnabled(isGyroEnabled);
			if (isGyroEnabled) {
				cout << "Gyroscope is enabled." << endl;
			}
			else {
				cout << "Gyroscope is disabled." << endl;
			}

		}
		bool isAccelEnabled;
		if (n.getParam("/recons_metric/isAccelEnabled",isAccelEnabled)) {
			me.setisAccelEnabled(isAccelEnabled);
			if (isAccelEnabled) {
				cout << "Accelerometer is enabled." << endl;
			}
			else {
				cout << "Accelerometer is disabled." << endl;
			}
		}
		bool isMagEnabled;
		if (n.getParam("/recons_metric/isMagEnabled",isMagEnabled)) {
			me.setisMagEnabled(isMagEnabled);
			if (isMagEnabled) {
				cout << "Magnetometer is enabled." << endl;
			}
			else {
				cout << "Magnetometer is disabled." << endl;
			}
		}
		bool isSLAMEnabled;
		if (n.getParam("/recons_metric/isSLAMEnabled",isSLAMEnabled)) {
			me.setisSLAMEnabled(isSLAMEnabled);
			if (isSLAMEnabled) {
				cout << "SLAM is enabled." << endl;
			}
			else {
				cout << "SLAM is disabled." << endl;
			}
		}

		bool isSLAMOrientationEnabled;
		if (n.getParam("/recons_metric/isSLAMOrientationEnabled",isSLAMOrientationEnabled)) {
			me.setisSLAMOrientationEnabled(isSLAMOrientationEnabled);
			if (isSLAMOrientationEnabled) {
				cout << "SLAM Orientation is enabled." << endl;
			}
			else {
				cout << "SLAM Orientation is disabled." << endl;
			}
		}

		bool isSLAMPositionEnabled;
		if (n.getParam("/recons_metric/isSLAMPositionEnabled",isSLAMPositionEnabled)) {
			me.setisSLAMPositionEnabled(isSLAMPositionEnabled);
			if (isSLAMPositionEnabled) {
				cout << "SLAM Position is enabled." << endl;
			}
			else {
				cout << "SLAM Position is disabled." << endl;
			}
		}

		bool isGPSEnabled;
		if (n.getParam("/recons_metric/isGPSEnabled",isGPSEnabled)) {
			me.setisGPSEnabled(isGPSEnabled);
			if (isGPSEnabled) {
				cout << "GPS is enabled." << endl;
			}
			else {
				cout << "GPS is disabled." << endl;
			}
		}

		bool isGPSSLAMThresholdEnabled;
		if (n.getParam("/recons_metric/isGPSSLAMThresholdEnabled",isGPSSLAMThresholdEnabled)) {
			me.setisGPSSLAMThresholdEnabled(isGPSSLAMThresholdEnabled);
			if (isGPSSLAMThresholdEnabled) {
				cout << "GPSSLAMThresholdEnabled is enabled." << endl;
			}
			else {
				cout << "GPSSLAMThresholdEnabled is disabled." << endl;
			}
		}


		me.setSensorObservationOptions(isGyroEnabled,isAccelEnabled,isMagEnabled,isSLAMEnabled,isGPSEnabled);

		if (isVargObtained && isVarbgObtained) {
//			me.setGyroVar(Vector3d::Map(&varg[0],3),Vector3d::Map(&varbg[0],3));
		}
		if (isVaraObtained && isVarbaObtained) {
			me.setAccVar(Vector3d::Map(&vara[0],3),Vector3d::Map(&varba[0],3));
		}
		if (isVarmObtained) {
			me.setMagVar(Vector3d::Map(&varm[0],3));
		}
		if (isVareSLAMObtained && isVarpSLAMObtained) {
			me.setSLAMVar(Vector3d::Map(&vareSLAM[0],3),Vector3d::Map(&varpSLAM[0],3));
		}
		if (isVarpGPSObtained) {
			me.setGPSVar(Vector3d::Map(&varpSLAM[0],3));
		}
		if (isVargObtained && isVarbgObtained && isVaraObtained && isVarbaObtained) {
			MatrixQ Q = MatrixQ::Zero();

			Q.block(0,0,3,3).diagonal() = Vector3d::Map(&varg[0],3);
			Q.block(3,3,3,3).diagonal() = Vector3d::Map(&varbg[0],3);
			Q.block(6,6,3,3).diagonal() = Vector3d::Map(&vara[0],3);
			Q.block(9,9,3,3).diagonal() = Vector3d::Map(&varba[0],3);
			kf.setMatrixQ(Q);
		}

//		if (isVarmObtained && isVaraObtained && isVarpGPSObtained && isVareSLAMObtained && isVarpSLAMObtained) {
//			MatrixR R = MatrixR::Zero();
//
//			R.block(0,0,3,3).diagonal() = Vector3d::Map(&varm[0],3);
//			R.block(3,3,3,3).diagonal() = Vector3d::Map(&vara[0],3);
//			R.block(6,6,3,3).diagonal() = Vector3d::Map(&vareSLAM[0],3);
//			R.block(9,9,3,3).diagonal() = Vector3d::Map(&varpSLAM[0],3);
//			R.block(12,12,3,3).diagonal() = Vector3d::Map(&varpGPS[0],3);
//			kf.setMatrixR(R);
//		}

		if (n.getParam("/recons_metric/logPath",logPath)) {
			cout << "logPath: " << logPath << endl;
		}

	}

	void System::imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message){
		//		cout << "imuCallback" << endl;
		double 	tIMU = imu_message->header.stamp.sec + imu_message->header.stamp.nsec*1e-9;
		if (me.isGyroEnabled()) {
			// Saving sequence
			//gyro.setDataSeq(imu_message->header.seq);
			// Saving Gyroscope raw message
			Vector3axes rawGyro(imu_message->angular_velocity.x, imu_message->angular_velocity.y, imu_message->angular_velocity.z);
			gyro.setDataRaw(rawGyro,tIMU,imu_message->header.seq);
		}
		if (me.isAccelEnabled()) {
		// Saving sequence
		// acc.setDataSeq(imu_message->header.seq);
		// Saving Accelerometer raw message
		Vector3axes rawAcc(imu_message->linear_acceleration.x, imu_message->linear_acceleration.y, imu_message->linear_acceleration.z);
		acc.setDataRaw(rawAcc,tIMU,imu_message->header.seq);
		}
		// Saving sequence
		imu.setDataSeq(imu_message->header.seq);
		// Saving IMU orientation
		VectorQuat orientationIMU(imu_message->orientation.w,imu_message->orientation.x,imu_message->orientation.y,imu_message->orientation.z);
		imu.setIMUOrientation(orientationIMU);

		if (gyro.isSensorStarted()) {
			poseEstimation();
			if (se.getScaleOption() == 2) {
				se.setRefPosition(gps.getDataPosition());
			}
			if (se.getScaleOption() == 3) {
				se.setRefValues(me.getEstOrientation(),me.getAccCorrected(),me.getEarthGravity(),me.getSampleTime());
			}
		}
		// Scaled Estimation
		if (se.getScaleOption() == 2 | se.getScaleOption() == 3) {
			if (slam.isSLAMStarted()) {
	//			se.setSLAMSampleTime(mag.getSampleTime());
				se.setSLAMPosition(slam.getDataPosition());
			}
			//se.setRefPosition(me.getEstPosition());
			se.scaleEstimation();
		}
	}

	void System::magCallback(const sensor_msgs::MagneticField::ConstPtr & mag_message){
//		cout << "magDataRaw" << endl;
		if (me.isMagEnabled()) {
	//		cout << "magCallback" << endl;
			double 	tMag = mag_message->header.stamp.sec + mag_message->header.stamp.nsec*1e-9;
			// Saving sequence
	//		mag.setDataSeq(mag_message->header.seq);
			// Saving raw message
			// T2G converts from Tesla to Gauss (message received in Tesla)
			Vector3axes rawMag(T2G*mag_message->magnetic_field.x,T2G*mag_message->magnetic_field.y,T2G*mag_message->magnetic_field.z);

			mag.setDataRaw(rawMag,tMag,mag_message->header.seq);
//			cout << "magDataRaw" << endl;
		}
	}

	void System::gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message){
		if (me.isGPSEnabled()) {
			// Saving Sequence
			gps.setDataSeq(gps_message->header.seq);

			// Saving GPS llh
			Vector3axes llh(gps_message->latitude*d2r, gps_message->longitude*d2r, gps_message->altitude);
			gps.setDataLLH(llh, gps_message->status.status);

			broadcasterTF(me.getEstOrientation(),gps.getDataPosition(),"world","gps");
			saveGPSCSVLog(gps.getDataPosition());
		}
	}

	void System::gpsExtFixCallback(const gps_common::GPSFix::ConstPtr & gps_message){
		if (me.isGPSEnabled()) {
			// Saving Sequence
			gps.setDataSeq(gps_message->header.seq);

			// Saving GPS llh
			Vector3axes llh(gps_message->latitude*d2r, gps_message->longitude*d2r, gps_message->altitude);
			gps.setDataLLH(llh, gps_message->status.status);
			gps.setDirection(gps_message->track*d2r);
			gps.setSpeed(gps_message->speed);

			// cout <<"Direction = "<< gps.getDirection() << endl;

			broadcasterTF(angle2quatZYX(-gps.getDirection(),0,0),gps.getDataPosition(),"world","gps");
			saveGPSCSVLog(gps.getDataPosition());
		}
	}

	void System::gpsRefFixCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message){
		// Saving Sequence
		gpsRef.setisGPSStarted(gps.isGPSStarted());
		gpsRef.setStartPosition(gps.getStartPosition());
		gpsRef.setDataSeq(gps_message->header.seq);
		// Saving GPS llhsetisNewData
		Vector3axes llh(gps_message->latitude*d2r, gps_message->longitude*d2r, gps_message->altitude);
		gpsRef.setDataLLH(llh, gps_message->status.status);
	}

	void System::gpsRefExtFixCallback(const gps_common::GPSFix::ConstPtr & gps_message){
//		cout << "gpsRefExtFixCallback" << endl;
		// Saving Sequence
		gpsRef.setisGPSStarted(gps.isGPSStarted());
		gpsRef.setStartPosition(gps.getStartPosition());
		gpsRef.setDataSeq(gps_message->header.seq);
		// Saving GPS llhsetisNewData
		Vector3axes llh(gps_message->latitude*d2r, gps_message->longitude*d2r, gps_message->altitude);
		gpsRef.setDataLLH(llh, gps_message->status.status);
		gpsRef.setDirection(gps_message->track*d2r);
		gpsRef.setSpeed(gps_message->speed);
		broadcasterTF(angle2quatZYX(-gpsRef.getDirection(),0,0),gpsRef.getDataPosition(),"world","ref");
	}

	void System::slamCallback(const geometry_msgs::PoseStamped::ConstPtr & slam_message) {
		if (me.isSLAMEnabled() | me.isSLAMOrientationEnabled() | me.isSLAMPositionEnabled()) {

//			&& slam.isSLAMStarted()
//            if (slam.getSampleTime() > 2 ) {
//    			cout << "dtSLAM = " <<  slam.getSampleTime() << endl;
////            	slam.setisSLAMStarted(false);
//            	slam.setisOffsetOrientationSet(false);
//            	slam.setisOffsetPositionSet(false);
//            	cout << "SLAM reset" << endl;
//            }
			double 	tSLAM = slam_message->header.stamp.sec + slam_message->header.stamp.nsec*1e-9;
			// Saving sequence
	//		slam.setDataSeq(slam_message->header.seq);
			VectorQuat orientationSLAM(slam_message->pose.orientation.w,slam_message->pose.orientation.x,slam_message->pose.orientation.y,slam_message->pose.orientation.z);
			Vector3axes positionSLAM(slam_message->pose.position.x,slam_message->pose.position.y,slam_message->pose.position.z);
			slam.setDataSLAM(orientationSLAM,positionSLAM, tSLAM, slam_message->header.seq);

			// Setting orientation offset
			if (me.isAttitudeStarted() && slam.isSLAMStarted() && !slam.isOffsetOrientationSet()) {
				slam.setOrientationOffset(me.getEstOrientation());
			}
			// Scaled Estimation

			if (slam.isSLAMStarted()) {
				se.setVelMagSLAM(slam.getVelMagFilt());
//				cout << "velMagSLAM = " << slam.getVelMagFilt() << endl;
				contVelMagSLAM ++;
//				cout << "contVelMagSLAM = " << contVelMagSLAM << endl;
				if (contVelMagSLAM == 100 && se.getAutoVelSLAMThreshold()) {
					se.setVelSLAMThreshold(1.05*slam.getVelMagFilt());
//					se.setVelSLAMThreshold(1.1*slam.getVelMagFilt());
					cout << "VelMagFilt = " << slam.getVelMagFilt() << endl;
					cout << "velSLAMThreshold = " << se.getVelSLAMThreshold() << endl;
				}
			}

			if (se.isScaleAvailable()) {
				publishScaleSLAM(se.getScale());
			}

			broadcasterTF(slam.getDataOrientation(),slam.getScaledPosition(),"world","slam");
			saveSLAMCSVLog(quat2angleZYX(slam.getDataOrientation()),slam.getScaledPosition());

		} //if (me.isSLAMEnabled() | me.isSLAMOrientationEnabled() | me.isSLAMPositionEnabled())
	} //void

	void System::poseEstimation(void) {
		// Reading sensor
		me.setAccValue(acc.getDataRaw());
		me.setMagValue(mag.getDataRaw());
		me.setGyroValue(gyro.getDataRaw());
		me.setGPSValue(gps.getDataPosition());
		me.setSLAMValue(slam.getDataOrientation(),slam.getScaledPosition());

		// Sample time changing
		me.setSampleTime(gyro.getSampleTime());


		// Is sensor Available or Reliable?
		me.setisSLAMAvailable(slam.isNewData() && me.isSLAMOrientationEnabled(), slam.isNewData() && slam.isScaleSet() && me.isSLAMPositionEnabled());
		me.setisGPSAvailable(gps.isNewData() && me.isGPSEnabled() && !slam.isScaleSet());
		me.setisMagAvailable( mag.isNewData() && mag.isInsideBounds() && me.isMagEnabled() && !slam.isOrientatioRunning());
		me.setisAccAvailable(acc.isNewData() && acc.isInsideBounds() && me.isAccelEnabled());

		if (gps.isGPSStarted() && slam.isSLAMStarted() && me.isGPSSLAMThresholdEnabled()) {
			// Is SLAM Reliable?
			double errorPosNorm = (gps.getDataPosition().segment(0,2) - slam.getScaledPosition().segment(0,2)).norm();
//			cout << "errorPosNorm =" << errorPosNorm << endl;
			bool isErrorPosOk = errorPosNorm < me.getPosGPSSLAMThreshold();
			if (!isErrorPosOk) {
				me.setisGPSAvailable(gps.isNewData() && me.isGPSEnabled());
				me.setisSLAMAvailable(slam.isNewData() && me.isSLAMOrientationEnabled(), false);
			}
		}

		// Set SLAM Scale
		if (se.getScaleOption() == 2 | se.getScaleOption() == 3 | se.getScaleOption() == 0) {
			slam.setisScaleSet(false);
		}
		if (se.isScaleAvailable() && slam.isSLAMStarted() && !slam.isScaleSet()) {
			slam.setScaleSLAM(se.getScale());
//			cout << "SLAM scale configured" << endl;
			// Setting position offset
			if (!slam.isOffsetPositionSet()) {
				slam.setPositionOffset(gps.getDataPosition());
//				slam.setPositionOffset(me.getEstPosition());
				cout << "SLAM offset configured" << endl;
			}
		}

		// Starting Attitude
		bool waitYaw;
		if (me.getYawOption() == 1) {
			waitYaw == !mag.isSensorStarted();
		}
		if (me.getYawOption() == 0) {
			waitYaw == false;
		}
		if (!me.isAttitudeStarted() && acc.isSensorStarted() && !waitYaw) { // && mag.isSensorStarted()
  			double rollAcc = acc.getRoll();
			double pitchAcc = acc.getPitch();
			double yawMag;
			switch (me.getYawOption()) {
				case 0: yawMag = 0.0;
				case 1: yawMag = mag.getYaw(rollAcc,pitchAcc);
			}

			me.startAttitude(rollAcc,pitchAcc,yawMag);
			me.setEarthGravity(acc.getGlobalValue());
			me.setEarthMagneticField(mag.getGlobalValue());
			kf.setKalmanType(1); //zError option
			ROS_INFO("Pose Estimation Started. ");
		}
		// Starting Position
		if (me.isAttitudeStarted() && !me.isPositionStarted()){
			contPos++; //
			// Position starting with GPS
			if (gps.isGPSStarted()) { // && contPos == contPosNumberIter
				me.startPosition(gps.getDataPosition());
			}
			// Position starting with SLAM
			if (me.isSLAMPositionAvailable()) { // && contPos == contPosNumberIter
				cout << "SLAM is Available" << endl;
				me.startPosition(slam.getScaledPosition());
			}
//			// Position starting with accelerometer
//			if (contPos == contPosNumberIter) {
//				me.setisPositionStarted(true);
//			}
		}
		//
		if (me.isAttitudeStarted() && gyro.isNewData() && acc.isNewData()) { // && me.isPositionStarted()   && mag.isNewData()
//			cout << "Kalman Filter" << endl;
			// ***** Kalman Filter *****

			//***** Prediction
			// Reset error state
			kf.setZeroState();

			// Propagation:  In this model is used the quaternion dynamic equation and the INS equations
			// to propagate the varsetisNewDataiables of the motion system
			me.propagation();

			kf.setMatrixF(me.getMatrixF());
			kf.setMatrixG(me.getMatrixG());
			kf.prediction();

			//**** Update
			kf.setObservationError(me.observationError());
			kf.setMatrixH(me.getMatrixH());
			kf.setMatrixR(me.getMatrixR());
			kf.update();

			me.state2variables(kf.getState());

			time_ros = ros::Time::now();

			publishOdometry(me.getEstOrientation(), me.getEstPosition(), me.getEstAngularVel(), me.getEstLinearVel());

			publishBiasAcc(me.getBiasAcc());

			publishBiasGyro(me.getBiasGyro());

			publishEstAngles(me.getEstAngles());

//
//			publishGyroAngles(me.getGyroAngles());
//
//			publishIMUAngles(imu.getIMUAngles());
//
//			publishPositionGPS(gps.getDataPosition());

			// Computing angles to publish
			double rollAcc = acc.getRoll();
			double pitchAcc = acc.getPitch();
			double yawMag = mag.getYaw(rollAcc,pitchAcc);
			Vector3axes anglesAccMag(rollAcc,pitchAcc,yawMag);
//			publishAccMagAngles(anglesAccMag);
//
//			publishSLAMAngles(quat2angleZYX(slam.getDataOrientation()));
//
//			publishPositionSLAM(slam.getDataPosition());

			broadcasterTF(me.getEstOrientation(),me.getEstPosition(),"world","local");
			saveOdometryCSVLog(me.getEstAngles(), me.getEstPosition(), me.getEstLinearVel(),me.getBiasGyro(),me.getBiasAcc());
//			saveGPSCSVLog(gps.getDasetisNewDatataPosition());
//			saveIMUCSVLog(imu.getIMUAngles());
//			saveGyroCSVLog(me.getGyroAngles());
//			saveSLAMCSVLog(quat2angleZYX(slam.getDataOrientation()),slam.getDataPosition());
//			saveAccMagCSVLog(anglesAccMag);
//			saveAccCSVLog(acc.getDataFiltered());
			saveResultsLog(quat2angleZYX(me.getEstOrientation()),imu.getIMUAngles(),quat2angleZYX(slam.getDataOrientation()),me.getEstPosition(),gps.getDataPosition(),slam.getScaledPosition(),se.getVelSLAMFiltered(),me.getAccCorrected(),gpsRef.getDataPosition(),me.getStateMarkov());
//			saveResultsLog(quat2angleZYX(me.getEstOrientation()),imu.getIMUAngles(),quat2angleZYX(slam.getDataOrientation()),me.getEstPosition(),gps.getDataPosition(),slam.getScaledPosition(),se.getVelSLAMFiltered(),me.getAccCorrected(),se.getRefPosition(),me.getStateMarkov());
//			saveResultsLog(me.getEstAngles(),imu.getIMUAngles(),quat2angleZYX(slam.getDataOrientation()),me.getEstPosition(),gps.getDataPosition(),slam.getScaledPosition(),se.getVelSLAMFiltered(),me.getAccCorrected(),se.getRefPosition());
			gyro.setisNewData(false);

			acc.setisNewData(false);
			mag.setisNewData(false);
			gps.setisNewData(false);
			slam.setisNewData(false);

			me.setisMotionEstimationStarted(true);
		}
	} // void poseEstimation(void)


	void System::publishOdometry(VectorQuat orientationValue, Vector3axes positionValue, Vector3axes linearVelocityValue, Vector3axes angularVelocityValue){
		nav_msgs::Odometry odometry_msg;
		odometry_msg.header.stamp = time_ros.now();
		odometry_msg.header.frame_id = "world";
		odometry_msg.pose.pose.position.x = positionValue(0);
		odometry_msg.pose.pose.position.y = positionValue(1);
		odometry_msg.pose.pose.position.z = positionValue(2);
		odometry_msg.pose.pose.orientation.w = orientationValue(0);
		odometry_msg.pose.pose.orientation.x = orientationValue(1);
		odometry_msg.pose.pose.orientation.y = orientationValue(2);
		odometry_msg.pose.pose.orientation.z = orientationValue(3);
		odometry_msg.twist.twist.linear.x = linearVelocityValue(0);
		odometry_msg.twist.twist.linear.y = linearVelocityValue(1);
		odometry_msg.twist.twist.linear.z = linearVelocityValue(2);
		odometry_msg.twist.twist.angular.x = angularVelocityValue(0);
		odometry_msg.twist.twist.angular.y = angularVelocityValue(1);
		odometry_msg.twist.twist.angular.z = angularVelocityValue(2);
		odometry_publisher.publish(odometry_msg);
	}

	void System::publishBiasAcc(Vector3axes biasAcc) {
		geometry_msgs::Vector3Stamped biasAcc_msg;
		biasAcc_msg.header.stamp = time_ros.now();
		biasAcc_msg.vector.x = biasAcc(0);
		biasAcc_msg.vector.y = biasAcc(1);
		biasAcc_msg.vector.z = biasAcc(2);
		biasAcc_publisher.publish(biasAcc_msg);
	}
	void System::publishBiasGyro(Vector3axes biasGyro) {
		geometry_msgs::Vector3Stamped biasGyro_msg;
		biasGyro_msg.header.stamp = time_ros.now();
		biasGyro_msg.vector.x = biasGyro(0);
		biasGyro_msg.vector.y = biasGyro(1);
		biasGyro_msg.vector.z = biasGyro(2);
		biasGyro_publisher.publish(biasGyro_msg);
	}
	void System::publishScaleSLAM(const double scaleSLAMValue) {
		geometry_msgs::Vector3Stamped scaleSLAM_msg;
		scaleSLAM_msg.header.stamp = time_ros.now();
		scaleSLAM_msg.vector.x = scaleSLAMValue;
		scaleSLAM_msg.vector.y = scaleSLAMValue;
		scaleSLAM_msg.vector.z = scaleSLAMValue;
		scaleSLAM_publisher.publish(scaleSLAM_msg);
	}

	void System::publishEstAngles(Vector3axes anglesEst) {
		geometry_msgs::Vector3Stamped anglesEst_msg;
		anglesEst_msg.header.stamp = time_ros.now();
		anglesEst_msg.vector.x = anglesEst(0);
		anglesEst_msg.vector.y = anglesEst(1);
		anglesEst_msg.vector.z = anglesEst(2);
		anglesEst_publisher.publish(anglesEst_msg);
	}

//	void System::publishIMUAngles(Vector3axes anglesIMU) {
//		geometry_msgs::Vector3 anglesIMU_msg;
//		anglesIMU_msg.x = anglesIMU(0);
//		anglesIMU_msg.y = anglesIMU(1);
//		anglesIMU_msg.z = anglesIMU(2);
//		anglesIMU_publisher.publish(anglesIMU_msg);
//	}
//
//	void System::publishGyroAngles(Vector3axes anglesGyro) {
//		geometry_msgs::Vector3 anglesGyro_msg;
//		anglesGyro_msg.x = anglesGyro(0);
//		anglesGyro_msg.y = anglesGyro(1);
//		anglesGyro_msg.z = anglesGyro(2);
//		anglesGyro_publisher.publish(anglesGyro_msg);
//	}
//
//	void System::publishAccMagAngles(Vector3axes anglesAccMag) {
//		geometry_msgs::Vector3 anglesAccMag_msg;
//		anglesAccMag_msg.x = anglesAccMag(0);
//		anglesAccMag_msg.y = anglesAccMag(1);
//		anglesAccMag_msg.z = anglesAccMag(2);
//		anglesAccMag_publisher.publish(anglesAccMag_msg);
//	}
//
//	void System::publishSLAMAngles(Vector3axes anglesSLAM) {
//		geometry_msgs::Vector3 anglesSLAM_msg;
//		anglesSLAM_msg.x = anglesSLAM(0);
//		anglesSLAM_msg.y = anglesSLAM(1);
//		anglesSLAM_msg.z = anglesSLAM(2);
//		anglesSLAM_publisher.publish(anglesSLAM_msg);
//	}
//
//	void System::publishPositionSLAM(Vector3axes GpSLAM) {
//		geometry_msgs::Point positionSLAM_msg;
//		positionSLAM_msg.x = GpSLAM(0);
//		positionSLAM_msg.y = GpSLAM(1);
//		positionSLAM_msg.z = GpSLAM(2);
//		positionSLAM_publisher.publish(positionSLAM_msg);
//	}
//
//	void System::publishPositionGPS(Vector3axes GpGPS) {
//		geometry_msgs::Point positionGPS_msg;
//		positionGPS_msg.x = GpGPS(0);
//		positionGPS_msg.y = GpGPS(1);
//		positionGPS_msg.z = GpGPS(2);
//		positionGPS_publisher.publish(positionGPS_msg);
//	}
//
	void System::broadcasterTF(VectorQuat orientation,Vector3axes position, string worldFrame, string localFrame) {
		static tf::TransformBroadcaster brTF;

		tf::Transform transformTF;
		transformTF.setOrigin( tf::Vector3(position(0),position(1), position(2)) );
		tf::Quaternion qTF;
		qTF.setValue(orientation(1),orientation(2),orientation(3),orientation(0));
		transformTF.setRotation(qTF);
		brTF.sendTransform(tf::StampedTransform(transformTF, time_ros.now(), worldFrame, localFrame));
	}

	void System::saveOdometryCSVLog(Vector3axes anglesHat, Vector3axes GpHat, Vector3axes GvHat, Vector3axes bgHat, Vector3axes baHat) {
		mefile<<setprecision(16)<<time_ros.now()<<"  " << anglesHat.transpose() <<"  "<< GpHat.transpose() <<"  "<< GvHat.transpose() <<"  "<<endl;
		mebiasfile<<setprecision(16)<<time_ros.now()<<"  "<<bgHat.transpose() <<"  "<< baHat.transpose() << endl;

	}
	void System::saveGPSCSVLog(Vector3axes posGPS) {
		gpsfile<<setprecision(16)<<time_ros.now()<<"  "<<posGPS.transpose()<< endl;
	}
//
//	void System::saveIMUCSVLog(Vector3axes anglesIMU) {
//		imufile<<setprecision(16)<<time_ros.now()<<" " << anglesIMU.transpose() << endl;
//	}
//
//	void System::saveGyroCSVLog(Vector3axes anglesGyro) {
//		gyrofile<<setprecision(16)<<time_ros.now()<<" " << anglesGyro.transpose() << endl;
//	}
	void System::saveSLAMCSVLog(Vector3axes anglesSLAM, Vector3axes positionSLAM) {
		slamfile<<setprecision(16)<<time_ros.now()<<" " << anglesSLAM.transpose() <<" "<< positionSLAM.transpose() << endl;
	}
//	void System::saveAccMagCSVLog(Vector3axes anglesAccMag) {
//		accmagfile<<setprecision(16)<<time_ros.now()<<" " << anglesAccMag.transpose() << endl;
//	}
//	void System::saveAccCSVLog(Vector3axes acc) {
//		accfile<<setprecision(16)<<time_ros.now()<<" " << acc.transpose() << endl;
//	}
//	void System::saveResultsLog(Vector3axes anglesHat,Vector3axes anglesIMU,Vector3axes anglesSLAM,Vector3axes GpHat,Vector3axes positionGPS,Vector3axes positionSLAM, double velSLAM, Vector3axes acc, Vector3axes GpRef, Vector3axes GpGPSRef) {
//		resultsfile<<setprecision(16)<<time_ros.now()<< " " << anglesHat.transpose() << " " << anglesIMU.transpose() << " " << anglesSLAM.transpose() << " " << GpHat.transpose() << " " << positionGPS.transpose() << " " << positionSLAM.transpose() << " " << velSLAM << " " << acc.transpose()  << " " << GpRef.transpose()  << " " << GpGPSRef.transpose()  << endl;
//	}
	void System::saveResultsLog(Vector3axes anglesHat,Vector3axes anglesIMU,Vector3axes anglesSLAM,Vector3axes GpHat,Vector3axes positionGPS,Vector3axes positionSLAM, double velSLAM, Vector3axes acc, Vector3axes GpRef, Vector3axes stateMarkov) {
		resultsfile<<setprecision(16)<<time_ros.now()<< " " << anglesHat.transpose() << " " << anglesIMU.transpose() << " " << anglesSLAM.transpose() << " " << GpHat.transpose() << " " << positionGPS.transpose() << " " << positionSLAM.transpose() << " " << velSLAM << " " << acc.transpose()  << " " << GpRef.transpose() << " " << stateMarkov.transpose() << endl;
	}
} // namespace ME
