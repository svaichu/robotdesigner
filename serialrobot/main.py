#!/usr/bin/env python3

from os import path, system, mkdir
from pydantic import BaseModel
from typing import List, Optional
import numpy as np
from numpy import sin, cos, pi, concatenate
from scipy.spatial.transform import Rotation as R
import subprocess
from subprocess import check_output, CalledProcessError
from urdf import urdf_write

class Workspace():
    def __init__(self):
        self.ws_path = "$HOME"
        self.createColconWS()
    def createColconWS(self):
        mkdir("my_ws")
        system("cd src")
    def createPackage(self):
        try:
            check_output(['bash', './createpackage.bash'])
            print("Bash script executed successfully")
        except CalledProcessError as e:
            print("Error occurred while executing bash script: ", e)

    def createRobot(self):
        self.robot = Robot()
    def createURDF(self):
        self.urdf_path = self.ws_path + package_path + "urdf"
    
    def colconBuildWS(self):
        try:
            check_output(['bash', './createpackage.bash'])
            print("Bash script executed successfully")
        except CalledProcessError as e:
            print("Error occurred while executing bash script: ", e)
        

class Link(BaseModel):
    id: Optional[int]= None
    name: Optional[str] = None
    theta: Optional[float] = 0
    d: Optional[float] = 0
    a: Optional[float] = 0
    alpha: Optional[float] = 0
    # parent: int = 0
    child: Optional['Link'] = None
    isLast: bool = False 
    isBase: bool = False 
    
    x: Optional[float] = 0
    y: Optional[float] = 0
    z: Optional[float] = 0
    roll: Optional[float] = 0
    pitch: Optional[float] = 0
    yaw: Optional[float] = 0
    ax: Optional[float] = 0
    ay: Optional[float] = 0
    az: Optional[float] = 0
    jx: Optional[float] = 0
    jy: Optional[float] = 0
    jz: Optional[float] = 0

    trans_mat: Optional[list] = 0

    def DH_trans(self):

        d, theta, a, alpha = self.d, self.theta, self.a, self.alpha 

        self.trans_mat = np.array([[cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha),    a*cos(theta)],
                            [sin(theta), cos(theta)*cos(alpha),    -1*cos(theta)*sin(alpha), a*sin(theta)],
                            [0,          sin(alpha),               cos(alpha),               d           ],
                            [0,          0,                        0,                        1           ]])

        return self.trans_mat   

class Robot(BaseModel):
    name: Optional[str] = None
    links: Optional[List[Link]] = []
    joint_frames: Optional[dict] = None
    dof: Optional[int] = 3

    def checkLimits(self):
        # TODO add limits

        pass

    def cypher(self, links, seed):
        for link in links:
            if link.isBase is True:
                seed[link.id] = link.trans_mat
            else:
                seed[link.id] = seed[link.id-1] @ link.trans_mat 
            yield seed

    
    def calculate_joint_frames(self):
        self.joint_frames = np.zeros((4, 4, 4))
        # self.joint_frames = [self.links[0].trans_mat]
        # self.joint_frames = [self.joint_frames[link.id-1] @ link.trans_mat if link.isBase is False else link.trans_mat for link in self.links]
        # for link in self.links:
        #     self.joint_frames.append(self.joint_frames[-1] @ link.trans_mat)
        list(self.cypher(self.links, self.joint_frames))
    

    def create_urdf(self, scale=1):
        self.links = [self.urdf_params(link) if link.isLast is False else link for link in self.links]
        # return(links_cp)

    def urdf_params(self, link):
        rpy =R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ')
        origins_vector = link.child.trans_mat[0:3,3]

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

            # print("axis is {}".format(axis))
            # print("angle is {}". format(angle))

            rpy = R.from_rotvec(angle * axis).as_euler('XYZ')

            print(rpy)
            print(cylinder_origin)
        
        link.roll = rpy[0]
        link.pitch = rpy[1]
        link.yaw = rpy[2]
        link.x = cylinder_origin[0]
        link.y = cylinder_origin[1]
        link.z = cylinder_origin[2]
        link.ax = self.joint_frames[link.id][0,2]
        link.ay = self.joint_frames[link.id][1,2]
        link.az = self.joint_frames[link.id][2,2]
        link.jx = link.trans_mat[0,3]
        link.jy = link.trans_mat[1,3]
        link.jz = link.trans_mat[2,3]
            # link.length = origins_vector_norm

        return(link)
