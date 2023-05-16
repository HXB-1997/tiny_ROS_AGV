#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2021 PS-Micro, Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#
from __future__ import division
import time

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from geometry_msgs.msg import Pose
def nothing(*arg):
        pass

icol = (18, 0, 196, 36, 255, 255)  # Yellow
cv2.namedWindow('colorTest')
# Lower range colour sliders.
cv2.createTrackbar('lowHue', 'colorTest', icol[0], 255, nothing)
cv2.createTrackbar('lowSat', 'colorTest', icol[1], 255, nothing)
cv2.createTrackbar('lowVal', 'colorTest', icol[2], 255, nothing)
# Higher range colour sliders.
cv2.createTrackbar('highHue', 'colorTest', icol[3], 255, nothing)
cv2.createTrackbar('highSat', 'colorTest', icol[4], 255, nothing)
cv2.createTrackbar('highVal', 'colorTest', icol[5], 255, nothing)

class ImageConverter:
    def __init__(self):
        # 创建图像缓存相关的变量
        self.cv_image = None
        self.get_image = False

        # 创建cv_bridge
        self.bridge = CvBridge()

        # 声明图像的发布者和订阅者
        
       
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",
                                          Image,
                                          self.callback,
                                          queue_size=1)

    def callback(self, data):
        # 判断当前图像是否处理完
        if not self.get_image:
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                show_image_raw = self.cv_image
                cv2.imshow('img2',show_image_raw)
                #invert perspective
                
                
                show_image=show_image_raw[300:480,0:450].copy()
               
                #cv2.imshow('img0',show_image)
                self.cv_image = show_image
                #raw_height, raw_width, channels = show_image.shape
                #print("raw_height, raw_width, channels",raw_height, raw_width, channels)
            except CvBridgeError as e:
                print e
            # 设置标志，表示收到图像
            self.get_image = True

    def detect_object(self):
        # 创建HSV阈值列表
        lowHue = cv2.getTrackbarPos('lowHue', 'colorTest')
        lowSat = cv2.getTrackbarPos('lowSat', 'colorTest')
        lowVal = cv2.getTrackbarPos('lowVal', 'colorTest')
        highHue = cv2.getTrackbarPos('highHue', 'colorTest')
        highSat = cv2.getTrackbarPos('highSat', 'colorTest')
        highVal = cv2.getTrackbarPos('highVal', 'colorTest')

        boundaries = [([lowHue, lowSat, lowVal], [highHue, highSat, highVal])]

        # 遍历HSV阈值列表
        for (lower, upper) in boundaries:
            # 创建HSV上下限位的阈值数组
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

        # 高斯滤波，对图像邻域内像素进行平滑
        hsv_image = cv2.GaussianBlur(self.cv_image, (5, 5), 0)

        # 颜色空间转换，将RGB图像转换成HSV图像
        hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2HSV)
        #cv2.imshow('image',hsv_image)
       
        # 根据阈值，去除背景
        mask = cv2.inRange(hsv_image, lower, upper)
        output = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

        # 将彩色图像转换成灰度图像
        cvImg = cv2.cvtColor(output, 6)  # cv2.COLOR_BGR2GRAY
        npImg = np.asarray(cvImg)
        thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]

        # 检测目标物体的轮廓
        thresh,cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,
                                                cv2.CHAIN_APPROX_NONE)

        # 遍历找到的所有轮廓线
        for c in cnts:

            # 去除一些面积太小的噪声
            if c.shape[0] < 150:
                continue

            # 提取轮廓的特征
            M = cv2.moments(c)

            if int(M["m00"]) not in range(500, 22500):
                continue

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            if cX > 360:
                 cX = 360 
            error_x = cX - 200
      
            print("x: {}, y: {}, error: {},size: {}".format(cX, cY,error_x, M["m00"]))

            # 把轮廓描绘出来，并绘制中心点
            cv2.drawContours(self.cv_image, [c], -1, (0, 0, 255), 2)
            cv2.circle(self.cv_image, (cX, cY), 1, (0, 0, 255), -1)
           
           
        
        final_image = self.cv_image
        cv2.imwrite(r'/home/agilex/Desktop/lena2.png',final_image)
        cv2.imshow('img',final_image)
        
        cv2.waitKey(10)
       

    def loop(self):
        if self.get_image:
            self.detect_object()
            self.get_image = False


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter = ImageConverter()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            image_converter.loop()
            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()
