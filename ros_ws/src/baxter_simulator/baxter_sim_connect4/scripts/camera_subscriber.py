import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

bridge = CvBridge()

def head_image_callback(msg):
    # head_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    print("head heard")

def left_image_callback(msg):
    # left_img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    print("left heard")

def Init():
    image_topic = '/cameras/head_camera/image/'
    rospy.Subscriber(image_topic, Image, head_image_callback)
    image_topic = '/cameras/left_hand_camera/image/'
    rospy.Subscriber(image_topic, Image, left_image_callback)

rospy.init_node('camera_listener')
head_caminfo=rospy.wait_for_message('cameras/head_camera/camera_info/',CameraInfo)
head_img = np.zeros((head_caminfo.height, head_caminfo.width, 3))
left_caminfo=rospy.wait_for_message('cameras/left_hand_camera/camera_info/',CameraInfo)
left_img = np.zeros((left_caminfo.height, left_caminfo.width, 3))
Init()
print(head_img.shape)
print(left_img.shape)

# msg = rospy.wait_for_message('/cameras/head_camera/camera_info/', CameraInfo)
# service calls for changing camera settings
# cameracontroller class cannot be used in simulator
# from sensor_msgs.srv import SetCameraInfo
# from sensor_msgs.msg import CameraInfo
# setCam = rospy.ServiceProxy('cameras/head_camera/set_camera_info', SetCameraInfo)
# rospy.wait_for_service('cameras/head_camera/set_camera_info', timeout=10)
# construct camInfoReq
# resp = ls(camInfoReq)
# if resp.success:
#     ...

import message_filters
   2 from sensor_msgs.msg import Image, CameraInfo
   3 
   4 def callback(image, camera_info):
   5   # Solve all of perception here...
   6 
   7 image_sub = message_filters.Subscriber('image', Image)
   8 info_sub = message_filters.Subscriber('camera_info', CameraInfo)
   9 
  10 ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
  11 ts.registerCallback(callback)
  12 rospy.spin()
