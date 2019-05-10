#!/usr/bin/env python
# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import unittest
from threading import Thread

import rospy

from pilz_robot_programming.robot import Robot
from brake_test_mock import BrakeTestMock

API_VERSION = "1"


class TestAPIBrakeTest(unittest.TestCase):
    """
    Test brake test support in the python api.
    """

    def setUp(self):
        self.robot = Robot(API_VERSION)

    def tearDown(self):
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None

    def test_timeout(self):
        """
        If method is called without service beeing available we expect a
        ROSException("timeout exceeded while waiting for service").
        """
        self.assertRaises(
            rospy.ROSException,
            Robot.is_brake_test_required,
            self.robot,
            1
        )

    def test_required(self):
        """
        We expect the method to return true, if the brake test is required.
        """
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_service()

        mock.set_is_braketest_required(True)
        try:
            res = self.robot.is_brake_test_required()
        except Exception as e:
            print(e)
        self.assertTrue(res)

        mock.stop()
        mock.join()

    def test_not_required(self):
        """
        We expect the method to return false, if the brake test is not required.
        """
        mock = BrakeTestMock()
        mock.start()
        mock.advertise_service()

        mock.set_is_braketest_required(False)
        try:
            res = self.robot.is_brake_test_required()
        except Exception as e:
            print(e)
        self.assertFalse(res)

        mock.stop()
        mock.join()


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_brake_test')
    rostest.rosrun('pilz_robot_programming', 'test_api_brake_test', TestAPIBrakeTest)
