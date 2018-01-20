# This file provides an abrstraction layer for the baxter interface
# Since the baxter interface has been written in python2, it is not compatible with python3.
# We use ZMQ to establich a connection between a python3 file and this python2 file.
# The commands sent from the python3 file will be captured by this python2 process and sent to the baxter interface.
# As a convention, we will use the tcp port 5555 to comunicate (same used in the ZMQ hello world).

import zmq
import rospy
import rospkg
import baxter_interface
from baxter_interface import CHECK_VERSION
from std_msgs.msg import(UInt16,)
from sensor_msgs.msg import Image, CameraInfo
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetLinkProperties, GetLinkProperties
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys
import  baxter_pb2  # avoid import all classes, will collide with ros classes.
import  gazebo_pb2
import time
import copy

def copyRosProto(src,dest):
	# util function to copy between proto pose and ros pose messages
	dest.position.x = src.position.x
	dest.position.y = src.position.y
	dest.position.z = src.position.z
	dest.orientation.x = src.orientation.x
	dest.orientation.y = src.orientation.y
	dest.orientation.z = src.orientation.z
	dest.orientation.w = src.orientation.w

class baxterServer:

	# ----------------------------------------------------------------------------------------
	# init
	def __init__(self, robot_ns, socketAddr):

		robot_ns = robot_ns.lstrip('/').rstrip('/')
		rospy.init_node("BaxterServer"+robot_ns)
		pub_rate = rospy.Publisher(robot_ns + '/joint_state_publish_rate', UInt16, queue_size=10)
		robot_ns = '/' + robot_ns + '/'
		self.robot_ns = robot_ns
		rs = baxter_interface.RobotEnable(robot_ns, CHECK_VERSION)
		init_state = rs.state().enabled
		rs.enable()

		# set joint state publishing to 100Hz
		pub_rate.publish(100) 

		# init - need to add all the variables that can be controlled!
		# the speed is set to 1,0 (max) by default - this should be controlled by the user!
		self.limbl = baxter_interface.Limb('left', robot_ns)
		self.limbl.set_joint_position_speed(1.0)
		self.limbr = baxter_interface.Limb('right', robot_ns)
		self.limbr.set_joint_position_speed(1.0)
		self.gravity = True
                self.gripperl = baxter_interface.Gripper('left', robot_ns)
                self.gripperr = baxter_interface.Gripper('right', robot_ns)
		self.head = baxter_interface.Head(robot_ns) # to keep head straight for vision
                self.bridge = CvBridge()
		image_topic = robot_ns + 'head_camera/image/'
		rospy.Subscriber(image_topic, Image, self.head_image_callback)
		image_topic = robot_ns + 'left_hand_camera/image/'
		rospy.Subscriber(image_topic, Image, self.left_image_callback)
                image_topic = robot_ns + 'right_hand_camera/image/'
                rospy.Subscriber(image_topic, Image, self.right_image_callback)
                topic =  '/gazebo/model_states'
                rospy.Subscriber(topic, ModelStates, self.gazebo_models_callback)
                self.model_keys=[]

		# get context and create socket for comunication
		self.context = zmq.Context()
		self.socket = self.context.socket(zmq.PAIR)
		self.socket.bind("tcp://*:%s"%(socketAddr))

	# ----------------------------------------------------------------------------------------
	# loop
	def loop(self):

		# loop until end message received
		endMessageReceived = False
		while endMessageReceived == False:

			#  Wait for next request from client, tell the client the message has been received
			message = self.socket.recv()
			endMessageReceived, answer = self.parseMessage(message)
                        if answer != '':
                            self.socket.send(answer)
                        else:
			    self.socket.send(b"Received")
                        


	# ----------------------------------------------------------------------------------------
	# parse messages received from the client - the comunication protocol is defined here!
	def parseMessage(self, message):

                endMessage = False
		answer = ''
		# I use the first three characters of the message to code the instruction.
		code = message[0:3]
                params = message[3:]

		# 000: end
		if code == '000':
			endMessage = True
                	self.deleteGazeboModels()
			self.cleanUpBaxter()
	
		# 001: baxter limb and gripper command 
		if code == '001':
                        command = baxter_pb2.BaxterCommand()
			mode = params[0]
                        command.ParseFromString(params[1:])
			self.baxterCommand(command, mode)

		# 002: baxter observation
                if code == '002':
			obs = self.baxterObserve(params)
                        answer = obs.SerializeToString()

                # 003: load gazebo models
		if code == '003':
			command = gazebo_pb2.GazeboCommand()
			command.ParseFromString(params)
			self.loadGazeboModels(command)

                # 004: change gazebo model position
		if code == '004':
			command = gazebo_pb2.GazeboCommand()
			command.ParseFromString(params)
			self.moveGazeboModels(command)

		# 005: observe gazebo models
		if code == '005':
			obs = self.gazeboObserve()
			answer = obs.SerializeToString()

		return endMessage, answer

	# --------------------------------------------------------------------------------------------
	# processing functions
	def disableGravity(self):
		robot_ns = self.robot_ns.rstrip('/')
		rospy.wait_for_service('gazebo/get_link_properties')
		get_lp = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
		rospy.wait_for_service('gazebo/set_link_properties')
		set_lp = rospy.ServiceProxy('gazebo/set_link_properties', SetLinkProperties)

		for link_name in ['::left_upper_shoulder', '::left_lower_shoulder', 
			'::left_upper_elbow', '::left_lower_elbow', 
			'::left_upper_forearm', '::left_lower_forearm', '::left_wrist',
			'::right_upper_shoulder', '::right_lower_shoulder', 
			'::right_upper_elbow', '::right_lower_elbow', 
			'::right_upper_forearm', '::right_lower_forearm', '::right_wrist']:
			resp_lp = get_lp(robot_ns + link_name)
			set_lp(robot_ns + link_name, resp_lp.com, False, resp_lp.mass, resp_lp.ixx, resp_lp.ixy, resp_lp.ixz,
				resp_lp.iyy, resp_lp.iyz, resp_lp.izz)


	def baxterCommand(self, command, mode):
                # find out side by "hasattr"
		self.disableGravity()
		if command.has_left:
                    side = 'left'
                    joint_command = command.left_joint_command
                    gripper_command = command.left_gripper_command
		else:
		    assert(command.has_right)
		    side = 'right'
                    joint_command = command.right_joint_command
                    gripper_command = command.right_gripper_command

                if side == 'left':
		    angles = self.limbl.joint_angles()
                else:
                    angles = self.limbr.joint_angles()

                if mode=='r':
			angles[side+'_s0'] += joint_command.s0 * np.pi / 180.0 
			angles[side+'_s1'] += joint_command.s1 * np.pi / 180.0
			angles[side+'_e0'] += joint_command.e0 * np.pi / 180.0
			angles[side+'_e1'] += joint_command.e1 * np.pi / 180.0
			angles[side+'_w0'] += joint_command.w0 * np.pi / 180.0
			angles[side+'_w1'] += joint_command.w1 * np.pi / 180.0
			angles[side+'_w2'] += joint_command.w2 * np.pi / 180.0
		else:
			angles[side+'_s0'] = joint_command.s0 * np.pi / 180.0
			angles[side+'_s1'] = joint_command.s1 * np.pi / 180.0
			angles[side+'_e0'] = joint_command.e0 * np.pi / 180.0
			angles[side+'_e1'] = joint_command.e1 * np.pi / 180.0
			angles[side+'_w0'] = joint_command.w0 * np.pi / 180.0
			angles[side+'_w1'] = joint_command.w1 * np.pi / 180.0
			angles[side+'_w2'] = joint_command.w2 * np.pi / 180.0

                if side == 'left':
		    if mode=='r':
			self.limbl.set_joint_positions(angles)
		    else:
			self.limbl.move_to_joint_positions(angles, timeout=5.0)
                    if gripper_command.command == baxter_pb2.GripperCommand.OPEN:
                        self.gripperl.open()
                    elif gripper_command.command == baxter_pb2.GripperCommand.CLOSE:
                        self.gripperl.close()
                else:
		    if mode=='r':
			self.limbr.set_joint_positions(angles)
		    else:
			self.limbr.move_to_joint_positions(angles, timeout=5.0)
                    if gripper_command.command == baxter_pb2.GripperCommand.OPEN:
                        self.gripperr.open()
                    elif gripper_command.command == baxter_pb2.GripperCommand.CLOSE:
                        self.gripperr.close()

	def baxterObserve(self, params):
		observation = baxter_pb2.BaxterObserve()
		if 'h' in params:
			observation.head_image.image = self.head_img.tostring()
			observation.head_image.height= self.head_img.shape[0]  # hopefully shape do not change in the middle
			observation.head_image.width = self.head_img.shape[1]
			observation.head_image.channel = self.head_img.shape[2]

		if 'l' in params:
			observation.left_hand_image.image = self.left_img.tostring()
			observation.left_hand_image.height= self.left_img.shape[0]
			observation.left_hand_image.width = self.left_img.shape[1]
			observation.left_hand_image.channel = self.left_img.shape[2]

		if 'r' in params:
			observation.right_hand_image.image = self.right_img.tostring()
			observation.right_hand_image.height= self.right_img.shape[0]
			observation.right_hand_image.width = self.right_img.shape[1]
			observation.right_hand_image.channel = self.right_img.shape[2]

    	        joint_angles = self.limbl.joint_angles()
		observation.left_joint_angles.s0 = joint_angles['left_s0']
		observation.left_joint_angles.s1 = joint_angles['left_s1']
		observation.left_joint_angles.e0 = joint_angles['left_e0']
		observation.left_joint_angles.e1 = joint_angles['left_e1']
		observation.left_joint_angles.w0 = joint_angles['left_w0']
		observation.left_joint_angles.w1 = joint_angles['left_w1']
		observation.left_joint_angles.w2 = joint_angles['left_w2']
		observation.left_gripper_state.state = baxter_pb2.GripperState.IDLE
		if self.gripperl.gripping():
		    observation.left_gripper_state.state = baxter_pb2.GripperState.SUCCESS			
		elif self.gripperl.position()<50:
		    observation.left_gripper_state.state = baxter_pb2.GripperState.FAIL
		end_effector = self.limbl.endpoint_pose()
		observation.left_endpose.position.x = end_effector['position'].x
		observation.left_endpose.position.y = end_effector['position'].y
		observation.left_endpose.position.z = end_effector['position'].z
		observation.left_endpose.orientation.x = end_effector['orientation'].x
		observation.left_endpose.orientation.y = end_effector['orientation'].y
		observation.left_endpose.orientation.z = end_effector['orientation'].z
		observation.left_endpose.orientation.w = end_effector['orientation'].w

    	        joint_angles = self.limbr.joint_angles()
		observation.right_joint_angles.s0 = joint_angles['right_s0']
		observation.right_joint_angles.s1 = joint_angles['right_s1']
		observation.right_joint_angles.e0 = joint_angles['right_e0']
		observation.right_joint_angles.e1 = joint_angles['right_e1']
		observation.right_joint_angles.w0 = joint_angles['right_w0']
		observation.right_joint_angles.w1 = joint_angles['right_w1']
		observation.right_joint_angles.w2 = joint_angles['right_w2']
		observation.right_gripper_state.state = baxter_pb2.GripperState.IDLE
		if self.gripperr.gripping():
		    observation.right_gripper_state.state = baxter_pb2.GripperState.SUCCESS			
		elif self.gripperr.position()<50:
		    observation.right_gripper_state.state = baxter_pb2.GripperState.FAIL
		end_effector = self.limbr.endpoint_pose()
		observation.right_endpose.position.x = end_effector['position'].x
		observation.right_endpose.position.y = end_effector['position'].y
		observation.right_endpose.position.z = end_effector['position'].z
		observation.right_endpose.orientation.x = end_effector['orientation'].x
		observation.right_endpose.orientation.y = end_effector['orientation'].y
		observation.right_endpose.orientation.z = end_effector['orientation'].z
		observation.right_endpose.orientation.w = end_effector['orientation'].w
		return observation

	def gazeboObserve(self):
		models = []
		robot_ns = self.robot_ns.lstrip('/').rstrip('/')
		for name, pose in zip(self.gazebo_msg.name, self.gazebo_msg.pose):
			if robot_ns in name:
				model = gazebo_pb2.ModelSpec()
				model.name = name
				copyRosProto(pose, model.pose)
				models.append(model)
		observation = gazebo_pb2.GazeboObserve()
		observation.models.extend(models)
		return observation

	def loadGazeboModels(self, command):
		# Get Models' Path
    		model_path = rospkg.RosPack().get_path('baxter_sim_connect4')+"/models/"
	        # Spawn Table SDF
	        rospy.wait_for_service('gazebo/spawn_sdf_model')
	        try:
	        	spawn_sdf = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
			for c in command.models:
				if c.path.endswith('sdf'):
					xml = ''
				        with open (model_path + c.path, "r") as modelfile:
				        	xml=modelfile.read().replace('\n', '')
					pose = Pose()
					copyRosProto(c.pose, pose)
        				resp_sdf = spawn_sdf(c.name, xml, "/", pose, 'world')
				        self.model_keys.append(c.name)
		except rospy.ServiceException, e:
		        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
		
		rospy.wait_for_service('gazebo/spawn_urdf_model')
	        try:
	        	spawn_urdf = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
			for c in command.models:
				if c.path.endswith('urdf'):
					xml = ''
				        with open (model_path + c.path, "r") as modelfile:
				        	xml=modelfile.read().replace('\n', '')
					pose = Pose()
					copyRosProto(c.pose, pose)
        				resp_urdf = spawn_urdf(c.name, xml, "/", pose, 'world')
				        self.model_keys.append(c.name)
		except rospy.ServiceException, e:
		        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


	def moveGazeboModels(self, command):
		rospy.wait_for_service('/gazebo/set_model_state')
		try:
			move_model = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
			for c in command.models:
				destModelState = ModelState()
				destModelState.model_name = c.name
				pose = Pose()
				copyRosProto(c.pose, pose)
				destModelState.pose = pose
				destModelState.twist = Twist()
				destModelState.reference_frame = 'world'
				resp = move_model(destModelState)
		except rospy.ServiceException, e:
		        rospy.logerr("Set model state service call failed: {0}".format(e))

	def deleteGazeboModels(self):
	    try:
	        delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
	        for key in self.model_keys:
	            resp_delete = delete_model(key)
	    except rospy.ServiceException, e:
	        rospy.loginfo("Delete Model service call failed: {0}".format(e))

	def cleanUpBaxter(self):
		pass

	def head_image_callback(self, msg):
		self.head_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

	def left_image_callback(self, msg):
		self.left_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        def right_image_callback(self, msg):
                self.right_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        def gazebo_models_callback(self, msg):
                self.gazebo_msg = copy.copy(msg)


# ----------------------------------------------------------------------------------------
# main - instantiate baxter server and loop
def main():
	robot_ns = sys.argv[1]
	sockerAddr = sys.argv[2]
	bs = baxterServer(robot_ns, sockerAddr)
	bs.loop()


# ----------------------------------------------------------------------------------------
# main
if __name__ == "__main__":
	main()

