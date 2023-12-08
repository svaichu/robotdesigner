import roboticstoolbox as rtb

robot  = rtb.models.Panda()

from os import path, system, mkdir
from pydantic import BaseModel
from typing import List, Optional
import numpy as np
from numpy import sin, cos, pi, concatenate
from scipy.spatial.transform import Rotation as R
import subprocess

def DH_trans(DH, joint_val):

     d, theta, a, alpha = (0,0,0,0)

     if (DH[0] == 'r'):

         d, theta, a, alpha = (DH[1], joint_val, DH[2], DH[3])

     elif (DH[0] == 'p'):

         d, theta, a, alpha = (joint_val, DH[1], DH[2], DH[3])

     elif (DH[0] == 'f'):

         d, theta, a, alpha = (DH[1], DH[2], DH[3], DH[4])

     trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                           [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                           [0,          sin(alpha),               cos(alpha),               d           ],
                           [0,          0,                        0,                        1           ]])

     return trans_mat

# DH Parameter Layout:
# ['r', d, a, alpha] for revolute joints
# ['p', theta, a, alpha] for prismatic joints
# ['f', d, theta, a, alpha] for fixed joints


def joint_transforms(DH_Params):

     transforms = []

     current_DOF = 0

     transforms.append(np.eye(4))

     for DH in DH_Params:
         
         if (DH[0] == 'r' or DH[0] == 'p'):
             transforms.append(DH_trans(DH, 0.0))
             current_DOF = current_DOF + 1

         else:
             transforms.append(DH_trans(DH, 0.0))

     return transforms

def joint_frames(transforms):
         
         joint_frames = [transforms[0]]
 
         for trans in transforms[1:]:
 
             joint_frames.append(joint_frames[-1] @ trans)
 
         return joint_frames

# DH Parameter Layout:
# ['r', d, a, alpha] for revolute joints
# ['p', theta, a, alpha] for prismatic joints
# ['f', d, theta, a, alpha] for fixed joints

def xml_string(DH_Params, scale=1):

    outstring = ""

    transforms = joint_transforms(DH_Params)

    frames = joint_frames(transforms)

    outstring = outstring + "<robot name='robot'>\n"

    outstring = outstring + "\t<material name='blue'>\n\t\t<color rgba='0 0 0.8 1'/>\n\t</material>\n"
    outstring = outstring + "\t<material name='red'>\n\t\t<color rgba='0.8 0 0 1'/>\n\t</material>\n"
    output =[]

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

class App():
    def __init__(self):
        self.ws_path = "$HOME"
        self.createColconWS()
    def createColconWS(self):
        mkdir("my_ws")
        system("cd src")

    def createRobot(self):
        self.robot = Robot()

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
    joint_frames: Optional[dict] = None

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
        self.joint_frames = {link.id: self.joint_frames @ link.trans_mat for link in self.links}
        return self.joint_frames

    def create_urdf(self, scale=1):
        output = {}
        # for i in range(len(transforms) - 1):
        # for link in self.links:
        #     if link.isLast is not True:
        #         el = link.trans_mat
        #         fr = self.joint_frames[link.id]

        #     # We need to create a cylinder to represent the joint
        #     # If the index is not zero, connect it to the previous link
        #     # And a joint to connect it to the link
        #     # And a box to connect the joints

        #         rpy = R.from_matrix(fr[0:3,0:3]).as_euler('XYZ')
        #         output = {link.id: [rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3]]}
            # output.append([rpy[0], rpy[1], rpy[2], el[0,3], el[1,3], el[2,3]])
        # output = {link.id: concatenate((R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ'), [link.trans_mat[0,3], link.trans_mat[1,3], link.trans_mat[2,3]])) for link in self.links if link.isLast is False}
        output = {link.id: concatenate((R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ'), [link.trans_mat[0,3], link.trans_mat[1,3], link.trans_mat[2,3]])) for link in self.links if link.isLast is False}        
        return(output)

    def urdf_params(self):
        R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ')