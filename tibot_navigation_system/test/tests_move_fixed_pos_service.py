#! /usr/bin/env python

import sys
sys.path.insert(0, '../src')
from move_fixed_pos_service import MoveFixedPos
from tibot_navigation_system.srv import MoveFixedPosMsg, MoveFixedPosMsgRequest, MoveFixedPosMsgResponse
import unittest


class CaseSrvCallback(unittest.TestCase):

    def setUp(self):
        self.mfp = MoveFixedPos()

    def runTest(self):
        response = self.mfp.srv_callback("place: bed")
        self.assertIsInstance(
            response, MoveFixedPosMsgResponse, "not are the same class!!")
        self.assertEquals(response, True, "Response is False!")


class CaseDegreeToRad(unittest.TestCase):

    def setUp(self):
        self.mfp = MoveFixedPos()

    def runTest(self):
        rad = self.mfp.degree_to_rad(90)
        self.assertEquals(round(rad, 2), 1.57, "!= 1.57")


class MyTestSuite(unittest.TestSuite):

    def __init__(self):
        super(MyTestSuite, self).__init__()
        self.addTest(CaseSrvCallback())
        self.addTest(CaseDegreeToRad())
