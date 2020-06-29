#!/usr/bin/env python

import sys, os
sys.path.insert(0, '/home/adil/sim_ws/src/task_5/kinematics')
from r3_ik import Inv_k
import unittest


class KineTest(unittest.TestCase):

    def setUp(self):
        self.testik = Inv_k(2,1,0,1,1,1)
        
    def test_theta2(self):
        self.testik.theta2_val()
        self.assertEqual(self.testik.wx,1)
        self.assertEqual(self.testik.wy,1)
        self.assertEqual(self.testik.t2,0)


    def test_theta1(self):
        self.testik.theta2_val()
        self.assertEqual(self.testik.theta1_val(),0)

    def test_theta3(self):
        self.testik.theta2_val()
        self.testik.theta1_val()
        self.assertEqual(self.testik.theta3_val(),-1.5707963267948966)


if __name__ == '__main__':
    unittest.main()
