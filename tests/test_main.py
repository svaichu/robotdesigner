import unittest
from serialrobot.main import Link, Robot
from pydantic import BaseModel
from numpy import sin, cos, pi, concatenate

class TestLink(unittest.TestCase):
    def test_link_creation(self):
        link = Link(id=1, name="Link1", theta=0.5, d=1.0, a=0.5, alpha=0.5, isLast=False, trans_mat=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        self.assertIsInstance(link, BaseModel)
        self.assertEqual(link.id, 1)
        self.assertEqual(link.name, "Link1")
        self.assertEqual(link.theta, 0.5)
        self.assertEqual(link.d, 1.0)
        self.assertEqual(link.a, 0.5)
        self.assertEqual(link.alpha, 0.5)
        self.assertEqual(link.isLast, False)
        self.assertEqual(link.trans_mat, [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])

class TestRobot(unittest.TestCase):
    def test_robot_creation(self):
        robot = Robot()
        self.assertIsInstance(robot, BaseModel)
        self.assertEqual(robot.links, [])
    
    def test_urdf(self):
        l0 = Link()
        l0.id = 0
        l0.d = 0
        l0.theta = 0
        l0.a = 0
        l0.alpha = 0

        l0.DH_trans()

        l1 = Link()

        l1.d = 2
        l1.id = 1
        l1.theta = pi/2
        l1.a = 2
        l1.alpha = 0
        l1.DH_trans()

        l2 = Link()
        l2.id = 2
        l2.d = 1
        l2.theta = 0
        l2.a = 1
        l2.alpha = 0

        l2.DH_trans()
        robot = Robot()
        robot.links = [l0, l1, l2]
        # robot.calculate_joint_frames()
        # self.assertEqual(robot.links, [])
        
if __name__ == '__main__':
    unittest.main()