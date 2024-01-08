from main import *

# l0 = Link()
# l0.id = 0
# l0.d = 0
# l0.theta = 0
# l0.a = 0
# l0.alpha = 0
# l0.isBase = True

# l0.DH_trans()

ws = Workspace()


robot = ws.createRobot("my_robot")
robot.addLink("base_link", 0, 0, 0, 0)
robot.addLink("shoulder", 2, 0, 3, 0)
robot.addLink("elbow", 2, 0, 1, 0)
robot.resolveLinks()
robot.calculate_joint_frames()
# ws.createWS()
# ws.createPackage()
# ws.createURDF()
# ws.createLaunch()
# ws.createRViz()
# ws.createSetup()
# ws.colconBuildWS()
robot.create_urdf()
ws.ws_path=Path.joinpath(Path.home(), "serialrobot", "serialrobot")
urdf_write(robot, Path.joinpath(ws.ws_path, "robot.urdf"))
