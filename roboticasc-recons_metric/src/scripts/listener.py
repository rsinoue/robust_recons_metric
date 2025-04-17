#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


class imuClass:
    flag = 0
    seq = 0.0
    t = 0.0
    ax = 0
    isDataUsed = 0
    
imu = imuClass() 
imuC

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.orientation.x)
    #rospy.loginfo("%f", data.orientation.x)
    #print "%f" %=data.orientation.x
    global imu
    imu.flag = 2
    seqPrevious = imu.seq
    imu.seq = data.header.seq
    imu.t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
    axPrevious = imu.ax
    imu.ax = data.linear_acceleration.x
    if (imu.ax != axPrevious):
        imu.isDataUsed = 0; # New data
    #else:
    #    imu.isDataUsed = 0;
        
    

    #print "%f" % imux

def listener():
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    global imu
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data",Imu, callback)
    #rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():   
        rospy.Subscriber("/imu/data",Imu, callback)   
        if imu.isDataUsed == 0: 
            print "seq: %i, t: %.10f" %  (imu.seq,imu.t)
            imu.isDataUsed = 1;
        elif imu.isDataUsed == 1:
            print "XXXXXXX %i" %  imu.seq
        else:
            print "AAAAAAA %f" %  imu.seq

            

        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
    #rate.sleep()
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.



    


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
