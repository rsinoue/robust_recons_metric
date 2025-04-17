/*
 * me.cpp
 *
 *  Created on: 6 May 2016
 *      Author: Roberto Santos Inoue
 */

#include "me_system.h"
#include "boost/thread.hpp"

using namespace ME;

int main(int argc, char **argv)
{
	// Initiate new ROS node named "recons_metric" motion_estimation_node
	ros::init(argc, argv, "motion_estimation");
//
	try {
		ME::System node;

//		ros::Rate loop_rate(100);
//		while (ros::ok())
//		{
			node.poseEstimation();
//		    ros::spinOnce();
//		    loop_rate.sleep();
//		}
		ros::spin();

	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}

  	return 0;
}



//ME::System node;
//
//
//void rosThreadLoop( int argc, char** argv)
//{
//	printf("Started ROS thread\n");
//
//    ros::NodeHandle n;
//
//    node.loadSettings(n);
//
//    node.loadTopics(n);
//
//	ros::spin();
//
//	ros::shutdown();
//
//	printf("Exiting ROS thread\n");
//
//	exit(1);
//}
//
//void rosFileLoop( int argc, char** argv)
//{
//
//	printf("Started ROS thread\n");
//
////	ros::Rate loop_rate(100);
////	while (ros::ok())
////	{
//		node.poseEstimation();
////		ros::spinOnce();
////		loop_rate.sleep();
////	}
//
//	ros::spin();
//
//	ros::shutdown();
//
//	printf("Exiting ROS thread\n");
//
//	exit(1);
//}
//
//
//int main(int argc, char **argv)
//{
//	// Initiate new ROS node named "recons_metric" motion_estimation_node
//	ros::init(argc, argv, "recons_metric");
//
//
//	boost::thread rosThread;
//
//	if(argc > 1)
//	{
//		rosThread = boost::thread(rosFileLoop, argc, argv);
//	}
//	else
//	{
//		// start ROS thread
//		rosThread = boost::thread(rosThreadLoop, argc, argv);
//	}
//
//
//	printf("Shutting down... \n");
//	ros::shutdown();
//	rosThread.join();
//	printf("Done. \n");
//}




