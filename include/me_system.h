/*
 * me_system.h
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#ifndef ME_SYSTEM_H
#define ME_SYSTEM_H


#include "ros/ros.h"
#include "sensors.h"
#include "me_definitions.h"
#include "kalman_filter.h"
#include "motion_estimation.h"
#include "scale_estimation.h"
#include <iostream>
#include <fstream>
#include "Eigen/Dense"


#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "gps_common/GPSFix.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
//#include "geometry_msgs/PointStamped.h"

// #include "nlopt.hpp"

using namespace std;

namespace ME {

	class System {

	    ros::NodeHandle n;
		ros::Publisher odometry_publisher;
		ros::Publisher biasAcc_publisher;
		ros::Publisher biasGyro_publisher;
		ros::Publisher scaleSLAM_publisher;
//		ros::Publisher anglesIMU_publisher;
		ros::Publisher anglesEst_publisher;
//		ros::Publisher anglesGyro_publisher;
//		ros::Publisher anglesAccMag_publisher;
//		ros::Publisher positionGPS_publisher;
//		ros::Publisher anglesSLAM_publisher;
//		ros::Publisher positionSLAM_publisher;
		ros::Subscriber imu_subscriber;
		ros::Subscriber mag_subscriber;
		ros::Subscriber gps_subscriber;
		ros::Subscriber gpsRef_subscriber;
		ros::Subscriber slam_subscriber;
		ros::Time time_ros;

		sensor_msgs::Imu imu_msg;
		sensor_msgs::MagneticField mag_msg;

		GYRO gyro;
		MAG mag;
		ACC acc;
		IMU imu;
		GPS gps;
		GPS gpsRef;
		SLAM slam;

		KalmanFilter kf;

		MotionEstimation me;

		ScaleEstimation se;

		int contPosNumberIter;
		int contPos;
		int contVelMagSLAM;

		double tIMU, tIMUPrevious;

		string logPath;

		// CSV log file
		ofstream mefile;
		ofstream mebiasfile;
//		ofstream imufile;
//		ofstream accmagfile;
//		ofstream accfile;
//		ofstream gyrofile;
		ofstream gpsfile;
		ofstream slamfile;
		ofstream resultsfile;

	  public:
		System();
		~System ();
		void openLogsFiles(const string &logPath);
		void closeLogsFiles(void);
		void loadTopics(ros::NodeHandle &n);
		void loadSettings(ros::NodeHandle &n);
		void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);
		void magCallback(const sensor_msgs::MagneticField::ConstPtr & mag_message);
		void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message);
		void gpsExtFixCallback(const gps_common::GPSFix::ConstPtr & gps_message);
		void gpsRefFixCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message);
		void gpsRefExtFixCallback(const gps_common::GPSFix::ConstPtr & gps_message);
		void slamCallback(const geometry_msgs::PoseStamped::ConstPtr & slam_message);
		void poseEstimation(void);
		void publishOdometry(VectorQuat orientationValue, Vector3axes positionValue, Vector3axes linearVelocityValue, Vector3axes angularVelocityValue);
		void publishBiasAcc(Vector3axes biasAcc);
		void publishBiasGyro(Vector3axes biasGyro);
		void publishScaleSLAM(const double scaleValue);
		void publishEstAngles(Vector3axes anglesHat);
//		void publishIMUAngles(Vector3axes anglesIMU);
//		void publishGyroAngles(Vector3axes anglesGyro);
//		void publishAccMagAngles(Vector3axes anglesAccMag);
//		void publishSLAMAngles(Vector3axes anglesSLAM);
//		void publishPositionSLAM(Vector3axes GpSLAM);
//		void publishPositionGPS(Vector3axes GpGPS);
//		void broadcasterMETF(VectorQuat orientation,Vector3axes position);
//		void broadcasterSLAMTF(VectorQuat orientation,Vector3axes position);
//		void broadcasterGPSTF(VectorQuat orientation,Vector3axes position);
		void broadcasterTF(VectorQuat orientation,Vector3axes position, string worldFrame, string localFrame);
		void saveOdometryCSVLog(Vector3axes anglesHat, Vector3axes GpHat, Vector3axes GvHat, Vector3axes bgHat, Vector3axes baHat);
		void saveGPSCSVLog(Vector3axes posGPS);
//		void saveIMUCSVLog(Vector3axes anglesIMU);
//		void saveGyroCSVLog(Vector3axes anglesGyro);
		void saveSLAMCSVLog(Vector3axes anglesSLAM, Vector3axes positionSLAM);
//		void saveAccMagCSVLog(Vector3axes anglesAccMag);
//		void saveAccCSVLog(Vector3axes acc);
//		void saveResultsLog(Vector3axes anglesHat,Vector3axes anglesIMU,Vector3axes anglesSLAM,Vector3axes GpHat,Vector3axes positionGPS,Vector3axes positionSLAM,double velSLAM, Vector3axes acc, Vector3axes GpRef, Vector3axes GpGPSRef);
		void saveResultsLog(Vector3axes anglesHat,Vector3axes anglesIMU,Vector3axes anglesSLAM,Vector3axes GpHat,Vector3axes positionGPS,Vector3axes positionSLAM,double velSLAM, Vector3axes acc, Vector3axes GpRef, Vector3axes stateMarkov);
	};


} // namespace ME

#endif // ME_SYSTEM_H
