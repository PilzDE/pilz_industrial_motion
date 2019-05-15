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


import rospy

from pilz_robot_programming.robot import *

_REQUIRED_API_VERSION = "1"


def start_program():
    print("Executing " + __file__)
    robot = Robot(_REQUIRED_API_VERSION)
    _test_wait_until_required(robot)


def _test_wait_until_required(robot):
    """Tests the brake test required.

        Test Sequence:
            1. Start robot motion.
            2. Emulate brake test required = True.
            3. Observe performed brake test.

        Expected Results:
            1. Robot stopped and prints to the console that no brake test is required.
            2. Robot starts brake test
            3. Brake test is successful.
    """

    while not robot.is_brake_test_required():
        rospy.sleep(1)
    robot.execute_brake_test()
    rospy.loginfo("Brake Test executed.")


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('acceptance_test_brake_test_node', anonymous=True)

    start_program()
