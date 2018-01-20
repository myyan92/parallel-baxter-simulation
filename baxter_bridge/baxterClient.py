# This is a python 3 class that wrapps around baxter commands and messages
# in order to work with gym environments and learning algorithms in python 3

import zmq
from baxter_pb2 import *
from gazebo_pb2 import *
from pose_pb2 import Pose
import time
import numpy as np
import subprocess
import socket

def convertPose(p):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    if len(p)>3:
        pose.orientation.x = p[3]
        pose.orientation.y = p[4]
        pose.orientation.z = p[5]
        pose.orientation.w = p[6]
    else:
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
    return pose

class BaxterClient():
    def __init__(self, robot_ns, side):
        self.side=side
        self.robot_ns = robot_ns

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('',0))
        addr = s.getsockname()
        s.close()
        print("using port %d" % (addr[1]))
        with open("/home/baxter_bridge/sockets.txt", "a") as f:
            f.write("%s %d\n" % (robot_ns, addr[1]))
        port = addr[1]

        self.context = zmq.Context()
        #  Socket to talk to server
        print("Connecting to server...")
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.connect("tcp://localhost:%d"%(port))

    def command(self, action, side=None, absolute=False):
        if side is None:
            side=self.side
        # encode side and joint angles, gripper command into message
        if absolute:
            message = b"001a"
        else:
            message = b"001r"
        command = BaxterCommand()
        if side=='left':
            command.left_joint_command.s0 = action[0]
            command.left_joint_command.s1 = action[1]
            command.left_joint_command.e0 = action[2]
            command.left_joint_command.e1 = action[3]
            command.left_joint_command.w0 = action[4]
            command.left_joint_command.w1 = action[5]
            command.left_joint_command.w2 = action[6]
            command.left_gripper_command.command = GripperCommand.STAY   
            # stay is default and may not show up when received
            if len(action)>7 and action[7] > 0.7:
                command.left_gripper_command.command = GripperCommand.CLOSE
            if len(action)>7 and action[7] < 0.3:
                command.left_gripper_command.command = GripperCommand.OPEN
            command.has_left = True
            command.has_right = False
        elif side=='right':
            command.right_joint_command.s0 = action[0]
            command.right_joint_command.s1 = action[1]
            command.right_joint_command.e0 = action[2]
            command.right_joint_command.e1 = action[3]
            command.right_joint_command.w0 = action[4]
            command.right_joint_command.w1 = action[5]
            command.right_joint_command.w2 = action[6]
            command.right_gripper_command.command = GripperCommand.STAY   
            # stay is default and may not show up when received
            if len(action)>7 and action[7] > 0.7:
                command.right_gripper_command.command = GripperCommand.CLOSE
            if len(action)>7 and action[7] < 0.3:
                command.right_gripper_command.command = GripperCommand.OPEN
            command.has_right=True
            command.has_left=False
        message = message + command.SerializeToString()
        self.socket.send(message)
        msg = self.socket.recv()
        
    def observe(self, side=None, req_head_image=False, req_hand_image=False):
        if side is None:
            side = self.side
        message = b"002"
        if req_head_image:
            message = message + b"h"
        if req_hand_image:
            message = message + side[0].encode('ascii')
        self.socket.send(message) # encode requested image to reduce unnecessary communication
        message = self.socket.recv()
        obs = BaxterObserve()
        obs.ParseFromString(message)
        result={}
        if req_head_image:
            head_image=np.fromstring(obs.head_image.image, dtype=np.uint8)
            head_image=head_image.reshape((obs.head_image.height, obs.head_image.width, obs.head_image.channel))
            result['head_image']=head_image
        if req_hand_image and side=='left':
            hand_image=np.fromstring(obs.left_hand_image.image, dtype=np.uint8)
            hand_image=hand_image.reshape((obs.left_hand_image.height, obs.left_hand_image.width, obs.left_hand_image.channel))
            result['hand_image']=hand_image
        if req_hand_image and side=='right':
            hand_image=np.fromstring(obs.right_hand_image.image, dtype=np.uint8)
            hand_image=hand_image.reshape((obs.right_hand_image.height, obs.right_hand_image.width, obs.right_hand_image.channel))
            result['hand_image']=hand_image
        if side=='left':
            limb=obs.left_joint_angles
            gripper=obs.left_gripper_state
            endpose=obs.left_endpose
        elif side=='right':
            limb=obs.right_joint_angles
            gripper=obs.right_gripper_state.state
            endpose=obs.right_endpose
        result['joint_angles']=[limb.s0,limb.s1,limb.e0,limb.e1,limb.w0,limb.w1,limb.w2]
        result['gripper']=0
        if gripper==GripperState.SUCCESS:
            result['gripper']=1
        elif gripper==GripperState.FAIL:
            result['gripper']=-1
        result['endpose']=endpose
        return result

    def load_gazebo_models(self, model_states):
        # model_states should be a dictionary with keys "name", "path", "pose". each pose should be a proto message
        command = GazeboCommand()
        models = []
        prefix = self.robot_ns.lstrip('/').rstrip('/')
        for name, path, pose in zip(model_states["name"], model_states["path"], model_states["pose"]):
            model = ModelSpec()
            if prefix in name:
                model.name = name
            else:
                model.name = prefix + '_' + name  # safety check to namespace models
            model.path = path
            if isinstance(pose, list):
                pose=convertPose(pose)
            model.pose.CopyFrom(pose)
            models.append(model)
        command.models.extend(models)
        message = command.SerializeToString()
        self.socket.send(b"003"+message)
        msg = self.socket.recv()
        
    def move_gazebo_models(self, model_states):
        # model_states should be a dictionary with keys "name", "pose". each pose should be a proto message
        command = GazeboCommand()
        models = []
        prefix = self.robot_ns.lstrip('/').rstrip('/')
        for name, pose in zip(model_states["name"], model_states["pose"]):
            model = ModelSpec()
            if prefix in name:
                model.name = name
            else:
                model.name = prefix + '_' + name  # safety check to namespace models
            if isinstance(pose, list):
                pose=convertPose(pose)
            model.pose.CopyFrom(pose)
            models.append(model)
        command.models.extend(models)
        message = command.SerializeToString()
        self.socket.send(b"004"+message)
        msg = self.socket.recv()

    def observe_gazebo_models(self):
        # can consider adding model name filters to reduce message length
        self.socket.send(b"005")
        message = self.socket.recv()
        obs = GazeboObserve()
        obs.ParseFromString(message)
        result={'name':[], 'pose':[]}
        for model in obs.models:
            result['name'].append(model.name)
            result['pose'].append(model.pose)
        return result

    def shutdown(self):
        # send a shutdown message so that server will delete gazebo models and do other cleanups.
        while True:
            try:
                self.socket.send(b"000")
                break
            except:
                pass

def main():
    baxter=BaxterClient('/alice', 'left')
#    baxter.command([5,5,5,5,5,5,5,1],side='right')
#    time.sleep(0.02)
#    obs = baxter.observe()
#    obs.head_image.image=''
#    obs.left_hand_image.image=''
#    print(obs)
    models = {'name':['cafe_table', 'block_1', 'block_2'],
              'path':['cafe_table/model.sdf', 'block/model.urdf', 'block/model.urdf'],
              'pose':[convertPose([1.0,0.0,-0.93]), convertPose([0.67,0.12,-0.14]), convertPose([0.77,0.12,-0.14])]}
    baxter.load_gazebo_models(models)
    obs = baxter.observe_gazebo_models()
    print(obs)
    time.sleep(5)
    models = {'name':['cafe_table', 'block_1', 'block_2'],
              'path':['cafe_table/model.sdf', 'block/model.urdf', 'block/model.urdf'],
              'pose':[convertPose([1.0,0.0,-0.93]), convertPose([0.0,-0.1,0.1]), convertPose([0.0,-0.1,0.1])]}
    baxter.move_gazebo_models(models)
    obs = baxter.observe_gazebo_models()
    print(obs)
    time.sleep(5)
    baxter.shutdown()

if __name__ == "__main__":
    main()

