#!/usr/bin/env python
'''
Baxter RSDK Inverse Kinematics Pick and Place for playing connect 4
'''
import argparse
import struct
import sys
import copy
import os

import rospy
import rospkg
import tf2_ros

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import baxter_interface

class camera_saver():
    def __init__(self):
        self.bridge = CvBridge()
        self.head_topic = '/cameras/head_camera/image/'
        self.left_topic = '/cameras/left_hand_camera/image/'
        self.save_path = os.path.join('.','baxter_image_board','')
        self.head_counter = 0
        self.left_counter = 0
        self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(30))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
    def save_head_image(self):
        msg = rospy.wait_for_message(self.head_topic, Image)
        t=msg.header.stamp
        try:
            trans = self.tfBuffer.lookup_transform('base', 'head_camera', t)
            with open(self.save_path+'head_%d.pose'%self.head_counter,'w') as f:
                trans.serialize(f)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("cannot get transformation info")
            return False
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(self.save_path+'head_%d.png'%self.head_counter, img)
        self.head_counter += 1
        return True
    def save_left_image(self):
        msg = rospy.wait_for_message(self.left_topic, Image)
        t=msg.header.stamp
        try:
            trans = self.tfBuffer.lookup_transform('base','left_hand_camera', t)
            with open(self.save_path+'left_%d.pose'%self.left_counter,'w') as f:
                trans.serialize(f)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("cannot get transformation info")
            return False
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite(self.save_path+'left_%d.png'%self.left_counter, img)
        self.left_counter += 1
        return True

class model_manager():
    def __init__(self):
        pass

    def load_gazebo_models(self, table_pose=Pose(position=Point(x=1.0, y=0.0, z=-0.925)),
                           table_reference_frame="base",
                           board_pose=Pose(position=Point(x=0.8725, y=0.0, z=-0.92)),
                           board_reference_frame="base"):
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('baxter_sim_connect4')+"/models/"
        # Load Table SDF
        table_xml = ''
        with open (model_path + "cafe_table/model.sdf", "r") as table_file:
            table_xml=table_file.read().replace('\n', '')
        # Load Board SDF
        board_xml = ''
        with open (model_path + "connect4_board/model.sdf", "r") as board_file:
            board_xml=board_file.read().replace('\n', '')
        # Load Ball URDF
        ball_red_xml = ''
        with open (model_path + "ball_blue_r/model.urdf", "r") as ball_file:
            ball_red_xml=ball_file.read().replace('\n', '')
        ball_yel_xml = ''
        with open (model_path + "ball_yellow_r/model.urdf", "r") as ball_file:
            ball_yel_xml=ball_file.read().replace('\n', '')
        # Spawn Table SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                                 table_pose, table_reference_frame)
            resp_sdf = spawn_sdf("connect4_board", board_xml, "/",
                                 board_pose, board_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        # Spawn Block URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            ball_reference_frame='base'
            ball_pose=copy.deepcopy(board_pose)
            ball_pose.position.z += 1.0
            ball_pose.position.x -= 0.005
            red_count = 0
            yellow_count = 0
            for i in xrange(7):
                ball_pose.position.y=board_pose.position.y+i*0.75*0.0254-0.07
                for j in xrange(10):
                    ball_color=np.random.randint(3)
                    if ball_color==0:
                        red_count += 1
                        resp_urdf = spawn_urdf("ball_red_%d"%(red_count), ball_red_xml, "/",
                                               ball_pose, ball_reference_frame)
                    elif ball_color==1:
                        yellow_count += 1
                        resp_urdf = spawn_urdf("ball_yellow_%d"%(yellow_count), ball_yel_xml, "/",
                                               ball_pose, ball_reference_frame)
                    else:
                        continue
                print red_count, yellow_count
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))
        self.red_count = red_count
        self.yellow_count = yellow_count

    def delete_gazebo_models(self):
        # This will be called on ROS Exit, deleting Gazebo models
        # Do not wait for the Gazebo Delete Model service, since
        # Gazebo should already be running. If the service is not
        # available since Gazebo has been killed, it is fine to error out
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("cafe_table")
            resp_delete = delete_model("connect4_board")
            for i in xrange(self.red_count):
                resp_delete = delete_model("ball_red_%d"%(i+1))
            for i in xrange(self.yellow_count):
                resp_delete = delete_model("ball_yellow_%d"%(i+1))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    model_mng=model_manager()
    model_mng.load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(model_mng.delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    limb = baxter_interface.Limb(limb)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()
    
    cam_saver = camera_saver()
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0,
                             'left_w1': -1.57,
                             'left_w2': 0,
                             'left_e0': 0,
                             'left_e1': 1.57,
                             'left_s0': -1.1,
                             'left_s1': 0}
    limb.move_to_joint_positions(starting_joint_angles)
    saved=False
    while not saved:
        saved = cam_saver.save_left_image()
    time.sleep(60)
    return 0

if __name__ == '__main__':
    sys.exit(main())
