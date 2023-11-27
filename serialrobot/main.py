import roboticstoolbox as rtb

robot  = rtb.models.Panda()

from os import path, system, mkdir

class App():
    def __init__(self):
        self.ws_path = "$HOME"
        self.createColconWS()
    def createColconWS(self):
        mkdir("my_ws")
        system("cd src")

    def createRobot(self):
        self.robot = Robot()

class Link():
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
