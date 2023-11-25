import roboticstoolbox as rtb

robot  = rtb.models.Panda()

from os import path

class App():
    def __init__(self):
        self.ws_path = "$HOME"
        self.createColconWS()
    def createColconWS(self):
        mkdir -p new_ws/src
        cd src
    def createRobot(self):
        self.robot = Robot()

class Link():
    def __init__(self, *args, **kwargs):
        self.id = arg[0]
        self.a=

class Robot():
    def __init__(self):
        self.a = []
        self.l = []
        self.d = []
        self.delta = []
    def checkLimits(self):
        # TODO add limits

        pass
    def generateURDF(self):

    def initROSpackage(self)
        #create colcon ws
        #build
