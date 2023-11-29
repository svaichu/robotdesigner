import roboticstoolbox as rtb

robot  = rtb.models.Panda()

from os import path, system, mkdir
from pydantic import BaseModel

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
    id: int
    name: str
    lmbda: int
    d: int
    l: int
    alpha: float
    parent: int = 0
    child: int = -1
    isLast: bool = True
    x: int = 0
    y: int = 0
    z: int = 0
    roll: int = 0
    pitch: int = 0
    yaw: int = 0
    
    def linkParams(self):
        self.z = self.d    
class Linkkk():
    def __init__(self, *args, **kwargs):
        self.id = args[0]
        self.parent_link = 0
        self.name = name
        self.lmbda = args[1]
        self.l = l
        self.d = d
        self.delta = delta
class Robot():
    def __init__(self):
        self.lmbda = []
        self.l = []
        self.d = []
        self.delta = []
    def checkLimits(self):
        # TODO add limits

        pass
    def generateURDF(self):
        self.robot_links = []
    def generate;launchfile():

    def initROSpackage(self):
        #create colcon ws
        mkdir("")
        #build
        return(ws_path)
