# import roboticstoolbox as rtb

# robot  = rtb.models.Panda()

from os import path, system, mkdir
from pydantic import BaseModel
from typing import List, Optional
import numpy as np
from numpy import sin, cos, pi, concatenate
from scipy.spatial.transform import Rotation as R
import subprocess
from urdf import URDF

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
    
    x: Optional[float] = 0
    y: Optional[float] = 0
    z: Optional[float] = 0
    roll: Optional[float] = 0
    pitch: Optional[float] = 0
    yaw: Optional[float] = 0
    ax: Optional[int] = 0
    ay: Optional[int] = 0
    az: Optional[int] = 0

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
    links: Optional[List[Link]] = []
    joint_frames: Optional[dict] = None

    def checkLimits(self):
        # TODO add limits

        pass

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
        self.links = [self.urdf_params(link) if link.isLast is False else link for link in self.links]
        # return(output)

    def urdf_params(self, link):
        R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ')
        origins_vector = self.links[link.id + 1].trans_mat[0:3,3]

        origins_vector_norm = np.linalg.norm(origins_vector)

        cylinder_origin = origins_vector/2

        rpy = [0, 0, 0]

        if (origins_vector_norm != 0.0):

            origins_vector_unit = origins_vector/origins_vector_norm

            axis = np.cross(origins_vector, np.array([0, 0, -1]))

            axis_norm = np.linalg.norm(axis)
            if (axis_norm != 0.0):
                axis = axis/np.linalg.norm(axis)

            angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))

            print("axis is {}".format(axis))
            print("angle is {}". format(angle))

            rpy = R.from_rotvec(angle * axis).as_euler('XYZ')
        
            link.roll = rpy[0]
            link.pitch = rpy[1]
            link.yaw = rpy[2]
            link.x = cylinder_origin[0]
            link.y = cylinder_origin[1]
            link.z = cylinder_origin[2]
            link.length = origins_vector_norm

        return(link)