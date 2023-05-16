#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
objPose = Pose()
objPose.position.x = 0
objPose.position.y = 0
objPose.position.z = 0

vel = Twist()
vel.linear.x = 0.0
vel.linear.y = 0.0
vel.linear.z = 0.0
vel.angular.x = 0.0
vel.angular.y = 0.0
vel.angular.z = 0.0


class follow_object:
    def __init__(self):    
        #订阅位姿信息

        self.Pose_sub = rospy.Subscriber("object_detect_pose", Pose, self.poseCallback)
        #发布速度指令
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def poseCallback(self,Pose):

        X = Pose.position.x;
        Y = Pose.position.y;
        Z = Pose.position.z;


        
        #self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))
      
        vel.angular.z = (100-X)/400
        z_out = vel.angular.z
        vel.linear.x = 0.2-abs(z_out)*0.07
        #if vel.angular.z < -0.25:
           #vel.angular.z = -0.25
        print("z_out = ",z_out)
        self.vel_pub.publish(vel)
        rospy.loginfo(
                    "Publsh velocity command[{} m/s, {} rad/s]".format(
                        vel.linear.x, vel.angular.z))

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("follow_object")
        rospy.loginfo("Starting follow object")
        follow_object()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down follow_object node."
        cv2.destroyAllWindows()
