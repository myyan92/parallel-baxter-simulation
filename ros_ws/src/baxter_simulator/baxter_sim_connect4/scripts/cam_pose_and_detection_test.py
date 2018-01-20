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

import baxter_interface

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

class Pose_with_shots(object):
    def __init__(self, limb, verbose=True):
        self._limb_name = limb # string
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        # camera image saver with tf poses
        self.cam_saver = camera_saver()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
        print "endpoint pose: "
        print self._limb.endpoint_pose()
        saved=False
        while not saved:
            saved = self.cam_saver.save_head_image()
        saved=False
        while not saved:
            saved = self.cam_saver.save_left_image()

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=-0.925)),
                       table_reference_frame="base",
                       board_pose=Pose(position=Point(x=0.6725, y=0.3265, z=-0.92)),
                       board_reference_frame="base",
                       ball_pose=Pose(position=Point(x=0.6725, y=0.1265, z=-0.14)),
                       ball_reference_frame="base"):
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
        for i in xrange(5):
            resp_urdf = spawn_urdf("ball_red_%d"%(i), ball_red_xml, "/",
                                   ball_pose, ball_reference_frame)
            ball_pose.position.y -= 0.2
            resp_urdf = spawn_urdf("ball_yellow_%d"%(i), ball_yel_xml, "/",
                                   ball_pose, ball_reference_frame)
            ball_pose.position.y += 0.2
            ball_pose.position.x += 0.1
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("connect4_board")
        for i in xrange(5):
            resp_delete = delete_model("ball_red_%d"%(i))
            resp_delete = delete_model("ball_yellow_%d"%(i))

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
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = Pose_with_shots(limb)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.1),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.0),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.1),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.2),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.3),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.4),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.35, z=0.4),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.35, z=0.3),
        orientation=overhead_orientation))
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    for pose in block_poses:
        pnp.servo_to_pose(pose)
    return 0

if __name__ == '__main__':
    sys.exit(main())
