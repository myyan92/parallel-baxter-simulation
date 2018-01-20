#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf2_ros
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo
import os

class camera_saver():
    def __init__(self):
        self.bridge = CvBridge()
        self.head_topic = '/cameras/head_camera/image/'
        self.left_topic = '/cameras/left_hand_camera/image/'
        self.save_path = os.path.join('.','baxter_image_trans','')
        self.head_counter = 0
        self.left_counter = 0
        self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
    def save_head_image(self):
        msg = rospy.wait_for_message(self.head_topic, Image)
        t=msg.header.stamp
        try:
            trans = self.tfBuffer.lookup_transform('head_camera','base', t)
            with open(self.save_path+'head_%d.pose'%self.head_counter,'w') as f:
                trans.serialize(f)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("cannot get transformation info")
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(self.save_path+'head_%d.png'%self.head_counter, img)
        self.head_counter += 1
    def save_left_image(self):
        msg = rospy.wait_for_message(self.left_topic, Image)
        t=msg.header.stamp
        try:
            trans = self.tfBuffer.lookup_transform('left_hand_camera','base', t)
            with open(self.save_path+'left_%d.pose'%self.left_counter,'w') as f:
                trans.serialize(f)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("cannot get transformation info")
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(self.save_path+'left_%d.png'%self.left_counter, img)
        self.left_counter += 1

rospy.init_node('camera_listener')
cam_saver = camera_saver()
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    cam_saver.save_head_image()
    cam_saver.save_left_image()
    rate.sleep()

