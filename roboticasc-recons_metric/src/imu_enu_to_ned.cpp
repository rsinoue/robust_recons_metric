/*
 * imu_enu_to_ned.cpp
 *
 *  Created on: 7 Jul 2016
 *      Author: Roberto Santos Inoue
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "Eigen/Dense"


#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

ros::Publisher imu_publisher;
ros::Publisher mag_publisher;

void imuCallback(const sensor_msgs::Imu::ConstPtr & imuENU_message){
    // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    tf::Quaternion quatENU;
    tf::quaternionMsgToTF(imuENU_message->orientation, quatENU);
    // The tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quatENU).getRPY(roll, pitch, yaw);

    tf::Quaternion quatNED = tf::createQuaternionFromRPY(roll, -pitch, -yaw);
    sensor_msgs::Imu imuNED_message;

    imuNED_message.orientation.x = quatNED[0];
    imuNED_message.orientation.y = quatNED[1];
    imuNED_message.orientation.z = quatNED[2];
    imuNED_message.orientation.w = quatNED[3];

    imuNED_message.linear_acceleration.x = imuENU_message->linear_acceleration.x;
    imuNED_message.linear_acceleration.y = -imuENU_message->linear_acceleration.y;
    imuNED_message.linear_acceleration.z = -imuENU_message->linear_acceleration.z;

    imuNED_message.angular_velocity.x = imuENU_message->angular_velocity.x;
    imuNED_message.angular_velocity.y = -imuENU_message->angular_velocity.y;
    imuNED_message.angular_velocity.z = -imuENU_message->angular_velocity.z;

    imuNED_message.header.stamp = imuENU_message->header.stamp;

    imu_publisher.publish(imuNED_message);

    // publish tf
   // broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), "world", "imu"));
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr & magENU_message){
	sensor_msgs::MagneticField magNED_message;

	magNED_message.magnetic_field.x = magENU_message->magnetic_field.x;
	magNED_message.magnetic_field.y = -magENU_message->magnetic_field.y;
	magNED_message.magnetic_field.z = -magENU_message->magnetic_field.z;

	magNED_message.header.stamp = magENU_message->header.stamp;

	mag_publisher.publish(magNED_message);

}

int main(int argc, char **argv)
{
	// Initiate new ROS node named "recons_metric" motion_estimation_node
	ros::init(argc, argv, "recons_metric");
	ros::NodeHandle n;
	ros::Subscriber imu_subscriber;
	ros::Subscriber mag_subscriber;
	tf::TransformBroadcaster broadcaster_;
	try {
		imu_publisher  = n.advertise<sensor_msgs::Imu>("/outIMU", 10);
		mag_publisher  = n.advertise<sensor_msgs::MagneticField>("/outMAG", 10);
		imu_subscriber = n.subscribe("/inIMU", 10, imuCallback);
		mag_subscriber = n.subscribe("/inMAG", 10, magCallback);
		ros::spin();
	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}
  	return 0;
}

