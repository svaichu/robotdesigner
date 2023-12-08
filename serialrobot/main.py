import roboticstoolbox as rtb

robot  = rtb.models.Panda()

from os import path, system, mkdir
from pydantic import BaseModel
from typing import List, Optional
import numpy as np
from numpy import sin, cos, pi
from scipy.spatial.transform import Rotation as R
import subprocess

class App():
    def __init__(self):
        self.ws_path = "$HOME"
        self.createColconWS()
    def createColconWS(self):
        mkdir("my_ws")
        system("cd src")

    def createRobot(self):
        self.robot = Robot()
def DH_trans(DH, joint_val):

    d, theta, a, alpha = (i for i in DH)

    trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                        [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                        [0,          sin(alpha),               cos(alpha),               d           ],
                        [0,          0,                        0,                        1           ]])

    return trans_mat

def link_transforms(DH_Params):

    transforms = []

    current_DOF = 0

    transforms.append(np.eye(4))

    for DH in DH_Params:
        
        transforms.append(DH_trans(DH, 0.0))
        current_DOF = current_DOF + 1

    return transforms

def joint_frames(transforms):
         
    joint_frames = [transforms[0]]

    for trans in transforms[1:]:

        joint_frames.append(joint_frames[-1] @ trans)

    return joint_frames

def xml_string(DH_Params, scale=1):
    transforms = link_transforms(DH_Params)
    frames = joint_frames(transforms)
    output = []
    for i in range(len(transforms) - 1):

        el = transforms[i]
        fr = frames[i]

        # We need to create a cylinder to represent the joint
        # If the index is not zero, connect it to the previous link
        # And a joint to connect it to the link
        # And a box to connect the joints

        rpy = R.from_matrix(fr[0:3,0:3]).as_euler('XYZ')
        output.append([rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3]])
    return(output)


class Link(BaseModel):
    id: Optional[int]= None
    name: Optional[str] = None
    theta: Optional[float] = 0
    d: Optional[float] = 0
    a: Optional[float] = 0
    alpha: Optional[float] = 0
    # parent: int = 0
    # child: int = -1
    isLast: bool = False 
    
    # x: int = 0
    # y: int = 0
    # z: int = 0
    # roll: int = 0
    # pitch: int = 0
    # yaw: int = 0

    trans_mat: Optional[list] = 0

    def DH_trans(self):

        d, theta, a, alpha = self.d, self.theta, self.a, self.alpha 

        self.trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                            [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                            [0,          sin(alpha),               cos(alpha),               d           ],
                            [0,          0,                        0,                        1           ]])

        return self.trans_mat
    
    def linkParams(self):
        self.z = self.l
        self.y = self.d
        self.r = 0

       

class Robot(BaseModel):
    name: Optional[str] = None
    links: Optional[List[Link]] = None
    def checkLimits(self):
        # TODO add limits

        pass
    def generateURDF(self):
        self.robot_links = []

# create function to run createpackage.bash fileimport subprocess

    def run_bash_script(self):
        try:
            subprocess.check_output(['bash', './createpackage.bash'])
            print("Bash script executed successfully")
        except subprocess.CalledProcessError as e:
            print("Error occurred while executing bash script: ", e)

    def calculate_joint_frames(self):
        self.joint_frames = np.eye(4)
        self.joint_frames = {link.id: self.joint_frames[-1] @ link.trans_mat for link in self.links if link.id != 0}
        return joint_frames

    def urdf_params(self, scale=1):
        frames = self.joint_frames()
        output = {}
        # for i in range(len(transforms) - 1):
        for link in self.links:
            if link.isLast is not True:
                el = link.trans_mat
                fr = self.joint_frames[link.id]

            # We need to create a cylinder to represent the joint
            # If the index is not zero, connect it to the previous link
            # And a joint to connect it to the link
            # And a box to connect the joints

            rpy = R.from_matrix(fr[0:3,0:3]).as_euler('XYZ')
            output = {link.id: [rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3]]}
            # output.append([rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3]])
        return(output)

def link_transforms(DH_Params):

    transforms = []

    current_DOF = 0

    transforms.append(np.eye(4))

    for DH in DH_Params:
        
        transforms.append(DH_trans(DH, 0.0))
        current_DOF = current_DOF + 1

    return transforms

def joint_frames(transforms):
         
    joint_frames = [transforms[0]]

    for trans in transforms[1:]:

        joint_frames.append(joint_frames[-1] @ trans)

    return joint_frames