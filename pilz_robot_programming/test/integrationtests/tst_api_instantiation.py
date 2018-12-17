#!/usr/bin/env python
# Copyright (c) 2018 Pilz GmbH & Co. KG
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
from pilz_robot_programming.robot import *

PLANNING_GROUP_NAME = "manipulator"
API_VERSION = "1"


class TestAPIInstantiation(unittest.TestCase):
    """ Test the Api Adapter Node. Result is evaluated from the error code in action result"""

    def test_wrong_version(self):
        """ Check that instance of Robot can not created with wrong version.

            Test sequence:
                1. Delete existing robot instance.
                2. Create robot instance with wrong version.

            Test Results:
                1. -
                2. Creation failed with RobotVersionError.

        """
        try:
            r = Robot("some_wrong_version")
        except RobotVersionError:
            pass
        else:
            del r
            self.fail('Robot instance can be created with wrong version.')

    def test_none_version(self):
        """ Check that instance of Robot can not be created with None version.

            Test sequence:
                1. Create robot instance with None version.

            Test Results:
                1. Creation failed with RobotVersionError.

        """
        try:
            r = Robot(None)
        except RobotVersionError:
            pass
        else:
            del r
            self.fail('Robot instance can be created with wrong version.')

    def test_multiple_instances(self):
        """ Check that multiple instances of Robot can not created.

            Test sequence:
                1. Create another instance of Robot.

            Test Results:
                1. Creation failed with RobotMultiInstancesError.
        """
        r1 = Robot(API_VERSION)
        try:
            r2 = Robot(API_VERSION)
        except RobotMultiInstancesError:
            del r1
        else:
            del r1, r2
            self.fail('Multiple robot instances does not throw exception.')


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_instantiation')
    rostest.rosrun('pilz_robot_programming', 'test_api_instantiation', TestAPIInstantiation)
