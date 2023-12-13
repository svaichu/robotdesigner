#!/usr/bin/env python3

from os import system, mkdir
from pathlib import Path
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
        self.ws_name = "my_ws" #TODO add name as argument
    def createWS(self):
        try:
            check_output(['bash', './createws.bash', self.ws_name])
            print("Bash script executed successfully")
            self.ws_path = Path.joinpath(Path.home(), self.ws_name)
        except CalledProcessError as e:
            print("Error occurred while executing bash script: ", e)
    def createRobot(self, name="robot"):
        l0 = Link()
        l0.id = 0
        l0.d = 0
        l0.theta = 0
        l0.a = 0
        l0.alpha = 0
        l0.isBase = True
        l0.DH_trans()
        self.robot = Robot(name=name) #TODO more robots per ws
        self.robot.links.append(l0)
        return self.robot
    
    def createPackage(self): #TODO createPackageForRobot, keep Robot() independent
        try:
            check_output(['bash', './createpackage.bash', self.robot.name])
            print("Bash script executed successfully")
        except CalledProcessError as e:
            print("Error occurred while executing bash script: ", e)

    def createURDF(self):
        self.urdf_path = Path.joinpath(self.ws_path, "src", self.robot.name, "urdf", "robot.urdf")
        self.robot.create_urdf()
        urdf_write(self.robot, self.urdf_path)
    
    def colconBuildWS(self):
        try:
            check_output(['bash', './colconbuildws.bash'])
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
    parent: Optional['Link'] = None
    child: Optional['Link'] = None
    isLast: bool = True 
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

    length: Optional[float] = 0

    jname: Optional[str] = None
    jtype: Optional[str] = None

    trans_mat: Optional[list] = 0

    def DH_trans(self):

        d, theta, a, alpha = self.d, self.theta, self.a, self.alpha #TODO fix naming

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

    def calculate_one_joint_frame(self, links, seed):
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
        list(self.calculate_one_joint_frame(self.links, self.joint_frames))
    

    def create_urdf(self, scale=1):
        outstring = ""
        outstring = outstring + "<robot name='robot'>\n"
        outstring = outstring + "\t<material name='blue'>\n\t\t<color rgba='0 0 0.8 1'/>\n\t</material>\n"
        outstring = outstring + "\t<material name='red'>\n\t\t<color rgba='0.8 0 0 1'/>\n\t</material>\n"

        # self.links = [self.urdf_params(link) for link in self.links]
        # return(links_cp)
        for link in self.links:
            if link.isLast is False:
                link, link_output = self.urdf_params(link)
                self.links[link.id] = link
                outstring = outstring + link_output
        outstring = outstring + "</robot>"
        # return outstring

    def urdf_params(self, link):
        rpy =R.from_matrix(self.joint_frames[link.id][0:3,0:3]).as_euler('XYZ')
        i = link.id
        outstring = ""
        
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

            print("axis is {}".format(axis))
            # print("angle is {}". format(angle))

            rpy = R.from_rotvec(angle * axis).as_euler('XYZ')

            # print(rpy)
            # print(cylinder_origin)
        outstring = outstring + "\t<link name='l{}'>\n".format(i)
        outstring = outstring + "\t\t<visual>\n"
        outstring = outstring + "\t\t\t<origin rpy='{} {} {}' xyz='{} {} {}'/>\n".format(rpy[0], rpy[1], rpy[2], cylinder_origin[0], cylinder_origin[1], cylinder_origin[2])
        outstring = outstring + "\t\t\t<geometry>\n"
        outstring = outstring + "\t\t\t\t<cylinder length='{}' radius='0.4'/>\n".format(origins_vector_norm) 
        outstring = outstring + "\t\t\t</geometry>\n"
        outstring = outstring + "\t\t\t<material name='red'/>\n"
        outstring = outstring + "\t\t</visual>\n"
        outstring = outstring + "\t</link>\n"

        # Add the actual joint between the cylinder and link

        jointType = "continuous"
        el = link.trans_mat
        fr = self.joint_frames[link.id]
        if(i != 0):
            outstring = outstring + "\t<joint name='move_l{}_from_a{}' type='{}'>\n".format(i, i, jointType)
            outstring = outstring + "\t\t<parent link='l{}'/>\n".format(i-1)
            outstring = outstring + "\t\t<child link='l{}'/>\n".format(i)
            outstring = outstring + "\t\t<axis xyz='{} {} {}'/>\n".format(fr[0,2], fr[1,2], fr[2,2])
            outstring = outstring + "\t\t<origin rpy='0 0 0' xyz='{} {} {}'/>\n".format(el[0,3], el[1,3], el[2,3])   
            outstring = outstring + "\t</joint>\n"
        
        link.roll = rpy[0] # trans_mat
        link.pitch = rpy[1]
        link.yaw = rpy[2]
        link.x = cylinder_origin[0]
        link.y = cylinder_origin[1]
        link.z = cylinder_origin[2]
        link.ax = self.joint_frames[link.id][0,2] #joint_frame
        link.ay = self.joint_frames[link.id][1,2]
        link.az = self.joint_frames[link.id][2,2]
        link.jx = link.trans_mat[0,3]
        link.jy = link.trans_mat[1,3]
        link.jz = link.trans_mat[2,3]
        link.length = origins_vector_norm

        link.jname = "move_l{}_from_a{}".format(i, i)
        link.jtype = jointType

        return(link, outstring)
    def addLink(self, name="l", d=0, theta=0, a=0, alpha=0):
        self.links[-1].isLast = False
        link = Link(id=len(self.links), d=d, theta=theta, a=a, alpha=alpha, name=name)
        link.DH_trans()
        self.links[-1].child = link
        link.parent = self.links[-1]
        self.links.append(link) #FEAT: add link as atrribute to Robot
        return link
