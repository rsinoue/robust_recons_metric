#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"



#include <iostream>
#include <fstream>

#include "teste/teste.h"

using namespace std;
using namespace Eigen;

const int sizeState = 15;
const int sizeObser = 12;
const int sizewNoise = 12;
const int sizevNoise = 12;
const int sizeQuat = 4;
const int size3axes = 3;

// Eigen types
typedef Matrix<double, sizeState, 1> VectorState;
typedef Matrix<double, sizeObser, 1> VectorObser;
typedef Matrix<double, sizeQuat, 1> VectorQuat;
typedef Matrix<double, size3axes, 1> Vector3axes;
typedef Matrix<double, sizeState, sizeState> MatrixF;
typedef Matrix<double, sizeState, sizewNoise> MatrixG;
typedef Matrix<double, sizeObser, sizeState> MatrixH;
typedef Matrix<double, sizeState, sizeState> MatrixP;
typedef Matrix<double, sizewNoise, sizewNoise> MatrixQ;
typedef Matrix<double, sizeState, sizeState> MatrixQd;
typedef Matrix<double, sizevNoise, sizevNoise> MatrixR;
typedef Matrix<double, sizeState, sizeObser> MatrixK;

// ROS variables
ros::Publisher odometry_publisher;
ros::Publisher vnAngles_publisher;
ros::Publisher meAngles_publisher;
ros::Publisher rateAngles_publisher;
ros::Publisher gpsPosition_publisher;
ros::Subscriber imu_subscriber;
ros::Subscriber mag_subscriber;
ros::Subscriber gps_subscriber;
ros::Subscriber slam_subscriber;
ros::Time time_ros;

// Constant variables
const float d2r = M_PI/180;
const float r2d = 180/M_PI;
const float T2G = 10e4;  	// Tesla to Gauss
const float dt = 0.01;   	// Sample time
const float fth = 0.3;   	// Threshold for acceleration integration see insPropagation function
const float alphaf = 0.99;  // Low pass filter variable see lowPassFilter function
const float magDec = 12.34*d2r;

// IMU variables

const double mmBoundThreshold = 0.2, aaBoundThreshold = 0.2;

enum sensorBoundsStatus {zeroRefBound = -1, outsideBounds = 0, insideBounds = 1};
sensorBoundsStatus aafBoundsSt = insideBounds;
sensorBoundsStatus mmfBoundsSt = insideBounds;


enum sensorStatus { notAvailable = 0, Available, Filtered, Used};
sensorStatus imuStatus = notAvailable;
sensorStatus magStatus = notAvailable;
sensorStatus aaStatus = notAvailable;
sensorStatus mmStatus = notAvailable;
sensorStatus omegagStatus = notAvailable;
sensorStatus gpsStatus = notAvailable;
sensorStatus slamStatus = notAvailable;

enum systemInit { notStarted = 0, Started, noData};
systemInit weightMatricesInit = notStarted;
systemInit attitudeIMUInit = notStarted;
systemInit attitudeMAGInit = notStarted;
systemInit attitudeInit = notStarted;
systemInit positionGPSInit = notStarted;
systemInit positionSLAMInit = notStarted;
systemInit positionInit = notStarted;
systemInit imuInit = notStarted;
systemInit magInit = notStarted;

VectorState x;
VectorState xp;
VectorObser z;
VectorObser hHat;
VectorObser zTil;
Vector3axes ge(0,0,0);
Vector3axes me(0,0,0);
Vector3axes aa(0,0,0);
Vector3axes aaf(0,0,0);
Vector3axes apHat(0,0,0);
Vector3axes apHatPrevious(0,0,0);
Vector3axes aHat(0,0,0);
Vector3axes deltabaHat(0,0,0);
Vector3axes baHat(0,0,0);
Vector3axes bapHat(0,0,0);
Vector3axes omegag(0,0,0);
Vector3axes omegagf(0,0,0);
Vector3axes omegagpHat(0,0,0);
Vector3axes omegagHat(0,0,0);
Vector3axes deltabgHat(0,0,0);
Vector3axes bgpHat(0,0,0);
Vector3axes bgHat(0,0,0);
Vector3axes mm(0,0,0);
Vector3axes mmf(0,0,0);
Vector3axes llhPrevious(0,0,0);
Vector3axes llh(0,0,0);
Vector3axes llh0(0,0,0);
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
nav_msgs::Odometry odometry_msg;
geometry_msgs::Vector3 meAngles_msg;
geometry_msgs::Vector3 vnAngles_msg;
geometry_msgs::Vector3 rateAngles_msg;
geometry_msgs::Point gpsPosition_msg;
VectorQuat LGqHat(0,0,0,0);
VectorQuat LGqpHat(0,0,0,0);
tf::Quaternion LGqHat_tf(0,0,0,0);
tf::Quaternion LGqpHat_tf(0,0,0,0);
tf::Quaternion vnQuat(0,0,0,0);
Vector3axes deltaAngle(0,0,0);
Vector3axes Angles(0,0,0);
Vector3axes rateAngles(0,0,0);
Vector3axes dAngles(0,0,0);
double rollAccel=0, pitchAccel=0, yawMag=0;


Vector3axes GvpHatPrevious(0,0,0);
Vector3axes GvpHat(0,0,0);
Vector3axes GppHat(0,0,0);

Vector3axes GvHatPrevious(0,0,0);
Vector3axes GvHat(0,0,0);
Vector3axes GpHat(0,0,0);
Vector3axes deltaGvHat(0,0,0);
Vector3axes deltaGpHat(0,0,0);

Vector3axes GvGPSPrevious(0,0,0);
Vector3axes GvGPS(0,0,0);
Vector3axes GpGPS(0,0,0);
Vector3axes GpGPS0(0,0,0);

Vector3axes GvSLAMPrevious(0,0,0);
Vector3axes GvSLAM(0,0,0);
Vector3axes GpSLAM(0,0,0);


bool isIMUavailable = false;
bool isMAGavailable = false;
bool isLFstarted = false;   // Is Local frame started?

double tmag = 0, tmagPrevious, dtmag;
double timu = 0, timuPrevious, dtimu;
double tkf = 0, tkfPrevious, dtkf;
double tgps = 0, tgpsPrevious, dtgps;

// Kalman filter variables
const int nMeanStartKf = 100;
int  contStartaa = 0, contStartmm = 0;
bool isKFinit = false;
int imuSeq, magSeq, gpsSeq, slamSeq;

MatrixF F;
MatrixG G;
MatrixH H;
MatrixP P;
MatrixP Pp;
MatrixQ Q;
MatrixQd Qd;
MatrixR R;
MatrixR S;
MatrixK K;
Matrix3d LGrotpHat;

double normP=0, normPPrevious=0;


// CSV log file
ofstream mefile;
ofstream mebiasfile;
ofstream vnfile;
ofstream ratefile;
ofstream gpsfile;

// Functions

void initKf(void);
void initWeightMatricesKF(void);
void initAccel(void);
void initMag(void);
void initAttitude(void);
void initGPS(void);
void initSLAM(void);
void initPosition(void);
void poseEstimation(void);
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);
void magCallback(const sensor_msgs::MagneticField::ConstPtr & mag_message);
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message);
Vector3axes lowPassFilter(Vector3axes vector, Vector3axes vectorPrevious, const float alpha);
void sensorBounds(enum sensorBoundsStatus &sB, Vector3d refVec, Vector3d Vec, const double bound);
VectorQuat quaternionPropagation(Vector3axes omega, VectorQuat q1, const float dt);
Matrix4d omegaMatrix(Vector3axes omega);
VectorQuat angle2quatZYX(double yaw, double pitch, double roll);
Vector3axes quat2angleZYX(VectorQuat q);
Vector3axes insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes LaPrevious, Vector3axes Gv, Vector3axes ge, const float dt, const float fth);
Vector3axes insPropagation(Vector3axes Gv, Vector3axes GvPrevious, Vector3axes Gp, const float dt);
Matrix3d quat2dcmZYX(VectorQuat q);
MatrixF montionSys15Matrices(VectorQuat LGq, Vector3axes omega, Vector3axes La, const float dt);
MatrixG montionSys15Matrices(VectorQuat LGq, const float dt);
MatrixH montionSys15Matrices(VectorQuat LGq, Vector3axes Gme, Vector3axes Gge);
Matrix3d crossMatrix(Vector3axes v);
VectorQuat deltaAngle2quat(Vector3axes deltaAngle, VectorQuat q1);
VectorQuat quatmultiply(VectorQuat q1, VectorQuat q2);
VectorQuat deltaAngle2quat(Vector3axes deltaAngle, VectorQuat q1);
VectorQuat quatmultiply(VectorQuat q1, VectorQuat q2);
Vector3d geodetic2ecef(Vector3d llh);
Vector3d ecef2ned(Vector3d pe, Vector3d llh0);


int main(int argc, char **argv)
{
    // CSV log files
	mefile.open("/home/roberto/Dropbox/3D Reconstruction/Projeto Pós-Doutorado - Roberto/MATLAB/cplusplusData/melog.csv");
	mebiasfile.open("/home/roberto/Dropbox/3D Reconstruction/Projeto Pós-Doutorado - Roberto/MATLAB/cplusplusData/mebiaslog.csv");
	gpsfile.open("/home/roberto/Dropbox/3D Reconstruction/Projeto Pós-Doutorado - Roberto/MATLAB/cplusplusData/gpslog.csv");
	vnfile.open("/home/roberto/Dropbox/3D Reconstruction/Projeto Pós-Doutorado - Roberto/MATLAB/cplusplusData/vnlog.csv");

	// Initiate new ROS node named "recons_metric" motion_estimation_node
	ros::init(argc, argv, "recons_metric");

    ros::NodeHandle n;

    gps_subscriber = n.subscribe("/fix", 1, gpsCallback);
    //slam_subscriber = n.subscribe("/orb_slam", 1, slamCallback);
    mag_subscriber = n.subscribe("/imu/mag", 1, magCallback);
    imu_subscriber = n.subscribe("/imu/data", 1, imuCallback);


    odometry_publisher = n.advertise<nav_msgs::Odometry>("/recons_metric/odometry",1);
    gpsPosition_publisher = n.advertise<geometry_msgs::Point>("recons_metric/gpsPosition",1);
    vnAngles_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/vnAngles",1);
    meAngles_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/meAngles",1);
    rateAngles_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/rateAngles",1);
    //slamAngles_publisher = n.advertise<geometry_msgs::Vector3>("/recons_metric/slamAngles",1);

    //poseEstimation();  // The function in this position has a problem of losing message from the topics, it has to be solved latter,
    // the function is in the magCallback function

  	ros::spin();


  	// Close CSV log file
  	mefile.close();
  	vnfile.close();
  	gpsfile.close();
  	return 0;
}

void initKf(void){
	initWeightMatricesKF();
	initAccel();
	initMag();
	initAttitude();
	initGPS();
	initSLAM();
	initPosition();
}



void initWeightMatricesKF(void) {
	if (weightMatricesInit == notStarted) {
		 ROS_INFO("Starting Kalman filter.");
			P = P.Identity();
			Q = Q.Zero();
			Q(0,0) =  0.01;  //vargx
			Q(1,1) =  0.01;  //vargy
			Q(2,2) =  0.01;  //vargz
			Q(3,3) =  0.00000001;  //varbgx
			Q(4,4) =  0.00000001;  //varbgy
			Q(5,5) =  0.00000001;  //varbgz
			Q(6,6) =  0.001;  //varax
			Q(7,7) =  0.001;  //varay
			Q(8,8) =  0.001;  //varaz
			Q(9,9) =  0.00000001;  //varbax
			Q(10,10) =  0.00000001; //varbay
			Q(11,11) =  0.00000001; //varbaz

			R =  R.Zero();
			R(0,0) =  0.001;     //varmx
			R(1,1) =  0.001;     //varmy
			R(2,2) =  0.001;     //varmz
			R(3,3) =  0.001;     //varax
			R(4,4) =  0.001;     //varay
			R(5,5) =  0.001;     //varaz
			R(6,6) =  1;      //varvGPSx
			R(7,7) =  1;      //varvGPSy
			R(8,8) =  1;      //varvGPSz
			R(9,9) =  0.00001;   //varpGPSx
			R(10,10) =  0.00001; //varpGPSy
			R(11,11) =  0.00001; //varpGPSz

			weightMatricesInit = Started;
	 }
}

void initAccel(void) {
	if (aaStatus == Filtered && attitudeIMUInit == notStarted) {
	    ge(2) = aaf.norm()*(1.0/((double) nMeanStartKf)) + ge(2);
    	//ge(2) = aaf(2)*(1.0/((double) nMeanStartKf)) + ge(2);
    	contStartaa++;
        //cout << "contStartaa = [" << contStartaa << "];" << endl;
    	if (contStartaa == nMeanStartKf) {
        	rollAccel = atan2(aaf(1),sqrt(pow(aaf(0),2)+ pow(aaf(2),2)));
        	pitchAccel = atan2(aaf(0),sqrt(pow(aaf(1),2) + pow(aaf(2),2)));

        	apHat = aaf;
        	aHat = aaf;

        	attitudeIMUInit = Started;
        	ROS_INFO("Accelerometers started. Estimated gravity vector ge = [%f, %f, %f]'.", ge(0),ge(1),ge(2));
        }
    }
}


void initMag(void){
    if (mmStatus == Filtered && attitudeMAGInit == notStarted) {
    	me(0) = mmf.norm()*(1.0/((double) nMeanStartKf)) + me(0);
        contStartmm++;
        //cout << "contStartmm = [" << contStartmm << "];" << endl;
        if (contStartmm == nMeanStartKf) {
         	yawMag = atan2(mmf(2)*sin(rollAccel) -(mmf(1)*cos(rollAccel)), mmf(0)*cos(pitchAccel) +mmf(1)*sin(pitchAccel)*sin(rollAccel) +mmf(2)*sin(pitchAccel)*cos(rollAccel) );
        	attitudeMAGInit = Started;
        	ROS_INFO("Magnetometers started. Estimated magnetic field vector me = [%f, %f, %f]'.", me(0),me(1),me(2));
        	//cout << "mmf = [" << mmf << "];" << endl;
        }
    }
    else if (contStartaa == nMeanStartKf && contStartmm == 0 && attitudeMAGInit == notStarted) {
    	yawMag = 0;
    	attitudeMAGInit = noData;
    }
}

void initAttitude(void){
    if (attitudeIMUInit == Started && (attitudeMAGInit == Started | attitudeMAGInit == noData) ) {
    	LGqHat = angle2quatZYX(yawMag,pitchAccel,rollAccel);
    	rateAngles(0) = rollAccel;
    	rateAngles(1) = pitchAccel;
    	rateAngles(2) = yawMag;

        // LGqHat_tf = tf::createQuaternionFromRPY(rollAccel,pitchAccel,yawMag);
        // cout << "LGqHat_tf = [" << LGqHat_tf.w() << " " << LGqHat_tf.x() << " "  << LGqHat_tf.y() << " " << LGqHat_tf.z() << "];" << endl;
        // cout << "IMU and Mag Angles started.";
        // cout << "LGqHat = [ \n\r" << LGqHat << "];" << endl;
    	attitudeInit= Started;
    	ROS_INFO("Attitude angles started.");
    }
}

void initGPS(void){
    if (gpsStatus == Available && attitudeInit == Started) {
    	GpHat = GpGPS;
    	positionGPSInit = Started;
    	ROS_INFO("Position initial values started based on GPS.");
    }
}

void initSLAM(void){
// to do

}

void initPosition(void){
// to do

}

void poseEstimation(void){
    //cout << "Pose Estimation" << endl;
    //ros::Rate loop_rate(100);
    //ros::NodeHandle node;
    //do {
      //  ros::spinOnce();

        if (attitudeInit == notStarted) {
            initKf();
        }
        else if ((attitudeInit == Started) && (omegagStatus == Filtered) && ( (aaStatus == Filtered)  | (mmStatus == Filtered) ) ){
        	//ros::shutdown();
            aaf = lowPassFilter(aa,aaf,alphaf);
            omegagf = lowPassFilter(omegag,omegagf,alphaf);
            imuStatus = Used;
            aaStatus = Filtered;
            omegagStatus = Filtered;

            mmf = lowPassFilter(mm,mmf,alphaf);
            magStatus = Used;
            mmStatus = Filtered;

            // Gyroscope
        	//omegagpHatPrevious = omegagpHat;
        	bgpHat = bgHat;
            omegagpHat = omegagf - bgpHat;

            // Accelerometer
            apHatPrevious = apHat;
            bapHat = baHat;
            apHat = aaf -bapHat;

            // Quaternion propagation
            LGqpHat = quaternionPropagation(omegagpHat, LGqHat,dt);
            //cout << "LGqpHat = [ \n\r" << LGqpHat << "];" << endl;
            // cout << "normLGqpHat = [" << LGqpHat.norm() << "];" << endl;

            // Vector3axes insPropagationLGqpHat(VectorQuat LGq, Vector3axes La, Vector3axes LaPrevious, Vector3axes Gv, Vector3axes ge, const float dt, const float fth)
            GvpHatPrevious = GvpHat;
            GvpHat = insPropagation(LGqpHat,apHat,apHatPrevious,GvHat,ge,dt,fth);
            //cout << "GvpHat= [ \n\r" << GvpHat << "];" << endl;
            GppHat = insPropagation(GvpHat, GvHatPrevious, GpHat, dt);

            //cout << "GpHat= [ \n\r" << GpHat << "];" << endl;

            // Reset state
            x = x.Zero();
            //cout << "x: \n\r" << x << endl;

            // Discrete-time system matrices
        	//cout << "imuSeq = [" << imuSeq << "];" << endl;
        	//cout << "magSeq = [" << magSeq << "];" << endl;
        	//cout << "gpsSeq = [" << gpsSeq << "];" << endl;
            // cout << "LGqHat = [ \n\r" << LGqHat << "];" << endl;
            // cout << "LGqpHat = [ \n\r" << LGqpHat << "];" << endl;
            // cout << "omegagpHat = [ \n\r" << omegagpHat <<  "];" << endl;
            // cout << "apHat = [ \n\r" << apHat << "];" << endl;
            // cout <<    ros::spinOnce(); "aa = [ \n\r" << aa << "];" << endl;
            // cout << "aaf = [ \n\r" << aa << "];" << endl;
            // cout << "mmf = [ \n\r" << mmf << "];" << endl;
            // cout << "ge = [ \n\r" << ge << "];" << endl;
            // cout << "me = [ \n\r" << me << "];" << endl;
            // cout << "dt = [ \n\r" << dt << "];" << endl;
            F = montionSys15Matrices(LGqpHat, omegagpHat, apHat, dt);
            //cout << "F = [ \n\r" << F << "];" << endl;
            G = montionSys15Matrices(LGqpHat, dt);
            //cout << "G = [ \n\r" << G << "];" << endl;
            // Kalman filter prediction
            // cout << "Q = [ \n\r" << Q << "];" << endl;
            // cout << "R = [ \n\r" << R << "];" << endl;
            Qd = G*Q*G.transpose();
            // cout << "Qd = [ \n\r" << Qd << "];" << endl;
            Pp = F*P*F.transpose() + Qd;
            // cout << "P = [ \n\r" << P << "];" << endl;
            // cout << "Pp = [ \n\r" << Pp << "];" << endl;


            // Kalman filter update
            LGrotpHat = quat2dcmZYX(LGqpHat);
            //z << mmf, aaf, Vector3d::Zero(), Vector3d::Zero();
            // z << mmf, apHat, Vector3d::Zero(), Vector3d::Zero();
            // cout << "z = [ \n\r" << z << "];" << endl;
            //hHat << LGrotpHat*me, -LGrotpHat*ge, Vector3d::Zero(), Vector3d::Zero();
            // cout << "hHat = [ \n\r" << hHat << "];" << endl;
            //zTil = z - hHat;
            // cout << "zTil = [ \n\r" << zTil << "];" << endl;

            if (mmStatus == Filtered && (mmfBoundsSt == insideBounds)) {
            	zTil.segment(0,3) = mmf - LGrotpHat*me;
            }
            else {
            	zTil.segment(0,3) = Vector3d::Zero();
            }
            if (aaStatus == Filtered && (aafBoundsSt == insideBounds)) {
            	zTil.segment(3,3) = apHat - (-LGrotpHat*ge);
            }
            else {
            	zTil.segment(3,3) = Vector3d::Zero();
            }
            if (slamStatus == Available && positionSLAMInit == Started) {
            	// To do
            	zTil.segment(6,3) = Vector3d::Zero();
            	slamStatus = notAvailable;
            }
            else {
            	zTil.segment(6,3) = Vector3d::Zero();
            }
            if (gpsStatus == Available && positionGPSInit == Started) {
            	zTil.segment(9,3) = GpGPS - GppHat;
            	// cout << "GPS is available" << endl;
            	// cout << "zTil = [" << zTil << "];" << endl;
            	// cout << "GpGPS = [" << GpGPS << "];" << endl;
            	// cout << "GppHat = [" << GppHat << "];" << endl;
        	   //ros::shutdown();
            }
            else {
            	zTil.segment(9,3) = Vector3d::Zero();
            	//cout << "GPS is not available" << endl;
            }

            H = montionSys15Matrices(LGqpHat, me, ge);
            //cout << "H = [ \n\r" << H << "];" << endl;
            S = H*Pp*H.transpose() + R;
            //cout << "S = [ \n\r" << S << "];" << endl;
            K = Pp*H.transpose()*S.inverse();
            //cout << "K = [ \n\r" << K << "];" << endl;
            xp = x;
            //cout << "xp = [ \n\r" << xp << "];" << endl;
            x = xp + K*zTil;
            // cout << "x = [ \n\r" << x << "];" << endl;
            P = (P.Identity() - K*H)*Pp;
            //cout << "P = [ \n\r" << P << "];" << endl;

            //normPPrevious = normP;
            //normP  = P.norm();
            //cout << "normP = [ \n\r" << normP<< "];" << endl;

            // Orientation computation
            deltaAngle = x.segment(0,3);
            // cout << "deltaAngle = [ \n\r" << deltaAngle << "];" << endl;
            LGqHat = deltaAngle2quat(deltaAngle, LGqHat);
            // cout << "normLGqpHat = [" << LGqpHat.norm() << "];" << endl;
            // cout << "normLGqHat = [" << LGqHat.norm() << "];" << endl;
            // VectorQuat LGqHat2    omegag(0) = imu_message->angular_velocity.x; = deltaAngle2quat(deltaAngle, LGqHat);
            // cout << "LGqHat2 = [ \n\r" << LGqHat2 << "];" << endl;
            Angles = quat2angleZYX(LGqHat);
            // Angles = quat2angleZYX(LGqHat2);
            // cout << "Angles = [ \n\r" << Angles*r2d << "];" << endl;

            // Checking the orientation quaternion norm
            if (LGqHat.norm() > 1) {
            	ROS_WARN("The norm of the orientation quaternion LGqHat is : %f.", LGqHat.norm());
            	//ros::shutdown();
            }

            // Gyro bias computation
            deltabgHat = x.segment(3,3);
            bgHat = bgHat + deltabgHat;
            omegagHat = omegagf - bgHat;
            //cout << "bgHat = [ \n\r" << bgHat<< "];" << endl;
            //cout << "omegagf = [ \n\r" << omegagf << "];" << endl;
            //cout << "omegagHat = [ \n\r" << omegagHat<< "];" << endl;
            // Accelerometer bias computation
            deltabaHat = x.segment(9,3);
            baHat = baHat + deltabaHat;
            aHat = aaf - baHat;
            //cout << "baHat = [ \n\r" << baHat<< "];" << endl;
            //cout << "aaf = [ \n\r" << aaf << "];" << endl;
            //cout << "aHat = [ \n\r" << aHat << "];" << endl;

            if (gpsStatus == Available && positionGPSInit == Started) {
            	// Position computation
            	deltaGpHat = x.segment(12,3);
            	GpHat = deltaGpHat + GpHat;
            	// Velocity computation
                deltaGvHat = x.segment(6,3);
                GvHat = deltaGvHat + GvHat;

            }
            else {
            	//cout << "Using INS propagation values" << endl;
            	GpHat = GppHat;
            	GvHat = GvpHat;
            }
        	// cout << "GpGPS = [" << GpGPS << "];" << endl;
        	// cout << "GpHat = [" << GpHat << "];" << endl;
        	// cout << "GvHat = [" << GvHat << "];" << endl;

            // Angular velocity computation
            dAngles = LGrotpHat.transpose()*omegagpHat;

            rateAngles = dAngles*dt + rateAngles;

        	tkfPrevious = tkf;
        	time_ros = ros::Time::now();
        	tkf =   time_ros.sec + time_ros.nsec*1e-9;
        	dtkf = tkf - tkfPrevious;

        	//cout << "dtkf = [" << dtkf << "];" << endl;

        	odometry_msg.header.stamp = ros::Time::now();
            odometry_msg.header.frame_id = "world";

            odometry_msg.pose.pose.position.x = GpHat(0);
            odometry_msg.pose.pose.position.y = GpHat(1);
            odometry_msg.pose.pose.position.z = GpHat(2);
            odometry_msg.pose.pose.orientation.w = LGqHat(0);
            odometry_msg.pose.pose.orientation.x = LGqHat(1);
            odometry_msg.pose.pose.orientation.y = LGqHat(2);
            odometry_msg.pose.pose.orientation.z = LGqHat(3);
            odometry_msg.twist.twist.linear.x = GvHat(0);
			odometry_msg.twist.twist.linear.y = GvHat(1);
			odometry_msg.twist.twist.linear.z = GvHat(2);
		    odometry_msg.twist.twist.angular.x = dAngles(0);
			odometry_msg.twist.twist.angular.y = dAngles(1);
			odometry_msg.twist.twist.angular.z = dAngles(2);

			odometry_publisher.publish(odometry_msg);

			meAngles_msg.x = Angles(0);
			meAngles_msg.y = Angles(1);
			meAngles_msg.z = Angles(2);

			rateAngles_msg.x = rateAngles(0);
			rateAngles_msg.y = rateAngles(1);
			rateAngles_msg.z = rateAngles(2);


			meAngles_publisher.publish(meAngles_msg);
			vnAngles_publisher.publish(vnAngles_msg);
			rateAngles_publisher.publish(rateAngles_msg);

			gpsPosition_msg.x = GpGPS(0);
			gpsPosition_msg.y = GpGPS(1);
			gpsPosition_msg.z = GpGPS(2);

			gpsPosition_publisher.publish(gpsPosition_msg);

			// Sensors status
			aaStatus = Used;
			omegagStatus = Used;
			mmStatus = Used;
    	    gpsStatus = Used;

			static tf::TransformBroadcaster brTF;
			tf::Transform transformTF;

			transformTF.setOrigin( tf::Vector3(GpHat(0),GpHat(1), GpHat(2)) );
			tf::Quaternion qTF;
			//q.setRPY(0, 0, msg->theta);
			qTF.setValue(LGqHat(1),LGqHat(2),LGqHat(3),LGqHat(0));
			transformTF.setRotation(qTF);

			brTF.sendTransform(tf::StampedTransform(transformTF, ros::Time::now(), "world", "local"));

			mefile<<setprecision(16)<<odometry_msg.header.stamp<<"  " << Angles(0) << "  "<< Angles(1) <<"  "<<Angles(2)<<"  "<< GpHat(0)<<"  "<<GpHat(1)<<"  "<< GpHat(2)<<"  "<<GvHat(0)<<"  "<<GvHat(1)<<"  "<< GvHat(2)<<"  "<<endl;

			mebiasfile<<setprecision(16)<<odometry_msg.header.stamp<<"  "<<bgHat(0)<<"  "<<bgHat(1)<<"  "<< bgHat(2)<<"  "<< baHat(0) << "  " << baHat(1) << "  "<<baHat(2)<< endl;

			vnfile<<setprecision(16)<<odometry_msg.header.stamp<<"  "<<vnAngles_msg.x<<"  " <<vnAngles_msg.y<<"  "<<vnAngles_msg.z<< endl;
			gpsfile<<setprecision(16)<<odometry_msg.header.stamp<<"  "<<GpGPS(0)<<"  "<<GpGPS(1)<<"  "<< GpGPS(2)<< endl;
            //cout << "ROS Time: " << ros::Time::now() << endl;
			//ros::shutdown();
			//loop_rate.sleep();
        }
    //}while(node.ok());
}

void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message){
	// Reading IMU data
	imuSeq = imu_message->header.seq;
    omegag(0) = imu_message->angular_velocity.x;
    omegag(1) = imu_message->angular_velocity.y;
    omegag(2) = imu_message->angular_velocity.z;
    aa(0) = imu_message->linear_acceleration.x;
    aa(1) = imu_message->linear_acceleration.y;
    aa(2) = imu_message->linear_acceleration.z;
    imuStatus = Available;
    // Low-pass filter
    // Set the initial condition for omegagf and aaf variables
    if (imuInit == notStarted) {
    	omegagf = omegag;
    	aaf = aa;
    	imuInit = Started;
    }
    aaf = lowPassFilter(aa,aaf,alphaf);
    aaStatus = Filtered;
    // Sensor bounds
    if (attitudeIMUInit == Started) {
		sensorBounds(aafBoundsSt, ge, aaf, aaBoundThreshold);
		if (aafBoundsSt == outsideBounds) {
			ROS_WARN("Accelerometer measurement norm is outside of the estipulated Threshold %f. aaf= [%f, %f, %f]', ge = [%f, %f, %f]',",aaBoundThreshold,aaf(0),aaf(1),aaf(2),ge(0),ge(1),ge(2));
		}
    }
    omegagf = lowPassFilter(omegag,omegagf,alphaf);
    omegagStatus = Filtered;

    // Getting orientation from VectorNav IMU - This block is only necessary to have a attitude reference
    tf::quaternionMsgToTF(imu_message->orientation,vnQuat);
    double roll, pitch, yaw;
    tf::Matrix3x3(vnQuat).getRPY(roll, pitch, yaw);
    vnAngles_msg.x = roll;
    vnAngles_msg.y = pitch;
    vnAngles_msg.z = yaw;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr & mag_message){
	// Reading Magnetometer data
	magSeq = mag_message->header.seq;
    mm(0) = T2G*mag_message->magnetic_field.x; // message received in Tesla, T2G converts Tesla to Gauss
    mm(1) = T2G*mag_message->magnetic_field.y;
    mm(2) = T2G*mag_message->magnetic_field.z;
    magStatus = Available;

    // Low-pass filter
    // Set the initial condition for mmf variable
    if (magInit == notStarted) {
    	mmf = mm;
    	magInit = Started;
    }
    mmf = lowPassFilter(mm,mmf,alphaf);
    mmf(2) = 0;
    mmStatus = Filtered;

    // Sensor bounds
    if (attitudeMAGInit == Started) {
		sensorBounds(mmfBoundsSt, me, mmf, mmBoundThreshold);
		if (mmfBoundsSt == outsideBounds) {
			ROS_WARN("Magnetometer measurement norm is outside of the estipulated Threshold %f. mmf= [%f, %f, %f]', me = [%f, %f, %f]',",mmBoundThreshold,mmf(0),mmf(1),mmf(2),me(0),me(1),me(2));
		}
    }

    // Sample time of the Magnetometer
	tmagPrevious = tmag;
	tmag = mag_message->header.stamp.sec + mag_message->header.stamp.nsec*1e-9;
	dtmag = tmag - tmagPrevious;

    poseEstimation(); // The Kalman filter is running in the same frequency of the magnetometer measurement
}


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_message){
	Vector3d ECEFp(0,0,0);
	Vector3d GpNED(0,0,0);
	VectorQuat qmagDec;
	Matrix3d RmagDec;

	qmagDec = angle2quatZYX(magDec,0,0);
    RmagDec = quat2dcmZYX(qmagDec);


	gpsSeq = gps_message->header.seq;
	llhPrevious = llh;
	llh(0) = gps_message->latitude*d2r;
	llh(1) = gps_message->longitude*d2r;
	llh(2) = gps_message->altitude;
    if (isKFinit == false) {
    	llhPrevious = llh;
    }
    tgpsPrevious = tgps;
    tgps = gps_message->header.stamp.sec + gps_message->header.stamp.nsec*1e-9;
	dtgps = tgps - tgpsPrevious;

	// LLH frame to Tangent frame
	if ( (gps_message->status.status != -1) && !isLFstarted) {

		llh0 = llh;
		ECEFp = geodetic2ecef(llh);
		GpGPS = RmagDec*ecef2ned(ECEFp, llh0);
		GpGPS0 = GpGPS;

		isLFstarted = true;
	    gpsStatus = Available;
	    ROS_INFO("Starting Tangent frame position.");

	}
	else if ( (gps_message->status.status != -1) && isLFstarted) {
		ECEFp = geodetic2ecef(llh);
        GpGPS = RmagDec*ecef2ned(ECEFp, llh0);
	    gpsStatus = Available;
	    //ROS_INFO("Computing Tangent frame position.");
	}
}

Vector3axes lowPassFilter(Vector3axes vector, Vector3axes vectorPrevious, const float alpha){
    Vector3axes vectorFiltered = alpha*vectorPrevious + (1-alpha)*vector;

    return vectorFiltered;
}

void sensorBounds(enum sensorBoundsStatus &sB, Vector3d refVec, Vector3d Vec, const double bound){
    sB = outsideBounds;
    double absError;
	// Sensor norm verfication
    double refVecNorm = refVec.norm();
    //cout << "refVecNorm = [" << refVecNorm << "]; " << endl;
    if (refVecNorm > 0) {
    	absError = abs(1 - Vec.norm()/refVecNorm);
    	if (absError <= bound) {
    		sB = insideBounds;
    	}
    }
    else {
    	sB= zeroRefBound;
    }
}

VectorQuat quaternionPropagation(Vector3axes omega, VectorQuat q1, const float dt) {
    Vector3axes w =0.5*omega*dt;
    Matrix4d OMEGA = omegaMatrix(w);
    double wAbs = w.norm();
    Matrix4d I4;
    I4 = I4.Identity();
    VectorQuat q2 = (cos(wAbs)*I4 + (sin(wAbs)/wAbs)*OMEGA)*q1;

    return q2;
}

Matrix4d omegaMatrix(Vector3axes omega) {
    Matrix4d OMEGA;
    OMEGA <<  0,       -omega(0), -omega(1), -omega(2),
              omega(0), 0,         omega(2), -omega(1),
              omega(1),-omega(2),  0,         omega(0),
              omega(2), omega(1), -omega(0),  0;

    return OMEGA;
}

VectorQuat angle2quatZYX(double yaw, double pitch, double roll){
    VectorQuat q; // q = [w, x, y, z]'
	q(0) = cos(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);
	q(1) = cos(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll) - sin(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll);
	q(2) = cos(0.5*yaw)*sin(0.5*pitch)*cos(0.5*roll) + sin(0.5*yaw)*cos(0.5*pitch)*sin(0.5*roll);
	q(3) = sin(0.5*yaw)*cos(0.5*pitch)*cos(0.5*roll) - cos(0.5*yaw)*sin(0.5*pitch)*sin(0.5*roll);

	return q;
}

Vector3axes quat2angleZYX(VectorQuat q){

	Vector3axes Angles;

	Angles(0) = atan2( 2*(q(2)*q(3) + q(0)*q(1)) , (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3)) ); // Roll angle - axis X
	Angles(1) = asin( -2*(q(1)*q(3) -q(0)*q(2)) );                                                    // Pitch angle - axis Y
	Angles(2) = atan2( 2*(q(1)*q(2) + q(0)*q(3)) , q(0)*q(0) + q(1)*q(1) - q(2)*q(2) - q(3)*q(3) );   // Yaw angle - axis Z

	return Angles;
}

Vector3axes insPropagation(VectorQuat LGq, Vector3axes La, Vector3axes LaPrevious, Vector3axes Gv, Vector3axes ge, const float dt, const float fth){
	// Trapezoidal integration uk = uk-1 + 0.5*dt(yk +yk-1)
	Matrix3d GLrot = quat2dcmZYX(LGq).transpose();
	Vector3axes f = 0.5*(GLrot*(La+LaPrevious)+2*ge);


	double absf0 = abs(f(0));
    if (absf0 < fth) {
	    f(0) = 0;
    }

	double absf1 = abs(f(1));
	if (absf1 < fth) {
	    f(1) = 0;
	}

	double absf2 = abs(f(2));
	if (absf2 < fth) {
	    f(2) = 0;
	}

	// Gv
	Gv = f*dt + Gv;
	return Gv;

}

Vector3axes insPropagation(Vector3axes Gv, Vector3axes GvPrevious, Vector3axes Gp, const float dt){
	// cout << "INS Propagation position" << endl;
	// Trapezoidal integration uk = uk-1 + 0.5*dt(yk +yk-1)
	// Gp
	Gp = 0.5*dt*(Gv + GvPrevious) + Gp;

	return Gp;
}

Matrix3d quat2dcmZYX(VectorQuat q) {
	Matrix3d R;
	R << q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)+q(0)*q(3)),             		   2*(q(1)*q(3)-q(0)*q(2)),
	     2*(q(1)*q(2)-q(0)*q(3)),         	      q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3),     2*(q(2)*q(3)+q(0)*q(1)),
	     2*(q(1)*q(3)+q(0)*q(2)),                 2*(q(2)*q(3)-q(0)*q(1)),                     q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);

	return R;
}


MatrixF montionSys15Matrices(VectorQuat LGq, Vector3axes omega, Vector3axes La, const float dt) {
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
	A.block(6,0,3,3) = minusRT*ACCELcross;
	A.block(6,9,3,3) = minusRT;
	A.block(12,6,3,3) = I3;

	F = Istate+A*dt;

	return F;
}

MatrixG montionSys15Matrices(VectorQuat LGq, const float dt) {
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

MatrixH montionSys15Matrices(VectorQuat LGq, Vector3axes Gme, Vector3axes Gge) {
	MatrixH H;
	H = H.Zero();
	Matrix3d R = quat2dcmZYX(LGq);

	Matrix3d GMEcross = crossMatrix(Gme);
	Matrix3d GGEcross = crossMatrix(Gge);
	Matrix3d I3 = I3.Identity();

	H.block(0,0,3,3) = R*GMEcross; 	// Magnetometer
	H.block(3,0,3,3) = -R*GGEcross; // Accelerometer
	H.block(3,9,3,3) = I3;			// Accelerometer bias
	H.block(6,6,3,3) = I3;          // Velocity
    H.block(9,12,3,3) = I3;         // Position

	return H;
}


Matrix3d crossMatrix(Vector3axes v){

	Matrix3d M;

	M << 0,   -v(2), v(1),
	     v(2), 0,   -v(0),
		-v(1), v(0), 0;

	return M;
}

VectorQuat deltaAngle2quat(Vector3axes deltaAngle, VectorQuat q) {
	VectorQuat s;
	VectorQuat deltaq;
    Vector3d deltaqVector = deltaAngle/2;
	double deltaqAux = deltaqVector.transpose()*deltaqVector;

	deltaq.segment(1,3) = deltaqVector;

	if (deltaqAux < 1)
		deltaq(0) = sqrt(1-deltaqAux);
	else
		deltaq(0) = sqrt(1+deltaqAux);
	s = quatmultiply(deltaq,q);

	return s;
}

VectorQuat quatmultiply(VectorQuat p, VectorQuat q) {
	Matrix4d  P;
	VectorQuat s;

	P << p(0), -p(1), -p(2), -p(3),
	     p(1),  p(0), -p(3),  p(2),
		 p(2),  p(3),  p(0), -p(1),
		 p(3), -p(2),  p(1),  p(0);

	s = P*q;

	return s;
}

Vector3d geodetic2ecef(Vector3d llh){
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

Vector3d ecef2ned(Vector3d pe, Vector3d llh0){
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
