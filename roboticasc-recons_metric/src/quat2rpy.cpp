/*
 * quat2rpy.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

/****************************************************************************
Conversion from a quaternion to roll, pitch and yaw.
****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"


// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:

ros::Publisher imu_rpy_publisher;
ros::Publisher re_rpy_publisher;
ros::Subscriber imu_quat_subscriber;
ros::Subscriber re_quat_subscriber;


// Function for conversion of quaternion to roll pitch and yaw.
// The angles are published here too.
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message){
    // The incoming geometry_msgs::Qrruaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_message->orientation, quat);
    // The tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // The found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // This Vector is then published:
    imu_rpy_publisher.publish(rpy);
    ROS_INFO("published imu rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

// Function for conversion of quaternion to roll pitch and yaw.
// The angles are published here too.
void reCallback(const nav_msgs::Odometry::ConstPtr & re_message){
    // The incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(re_message->pose.pose.orientation, quat);
    // The tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // The found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // This Vector is then published:
    re_rpy_publisher.publish(rpy);
    ROS_INFO("published re rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

/*
// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const geometry_msgs::Quaternion msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    // this Vector is then published:
    imu_rpy_publisher.publish(rpy);
    ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    imu_rpy_publisher = n.advertise<geometry_msgs::Vector3>("imu_rpy_angles", 10);
    re_rpy_publisher = n.advertise<geometry_msgs::Vector3>("re_rpy_angles", 10);
    imu_quat_subscriber = n.subscribe("imu/data", 10, imuCallback);
    re_quat_subscriber = n.subscribe("recons_metric/odometry", 10, reCallback);
    // Check for incoming quaternions untill ctrl+c is pressed
    ROS_INFO("waiting for quaternion");
    ros::spin();
    return 0;
}
