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

import rospy
import unittest
import subprocess
import signal
from rospkg import RosPack
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_industrial_motion_testutils.robot_motion_observer import RobotMotionObserver
from pilz_robot_programming.commands import *

_TEST_DATA_FILE_NAME = RosPack().get_path("pilz_industrial_motion_testutils") + "/test_data/testdata.xml"
_GROUP_NAME = "manipulator"


class TestAPIProgramKill(unittest.TestCase):
    """
    Test the API behaviour when the python program is killed.
    """

    _PTP_TEST_NAME = "PTPJointValid"

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.robot_motion_observer = RobotMotionObserver(_GROUP_NAME)

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        if hasattr(self, 'test_data'):
            del self.test_data

    def _get_robot_move_command(self):
        """Build command to move the robot (to be called as subprocess)

        :returns list containing executable with path and appropriate arguments
        """
        ptp_goal = [str(x) for x in self.test_data.get_joints(self._PTP_TEST_NAME, _GROUP_NAME)]
        movecmd = RosPack().get_path("pilz_robot_programming")+'/test/integrationtests/movecmd.py ptp joint'

        return movecmd.split(" ") + ptp_goal

    def test_stop_at_program_kill(self):
        """
        Test if robot movement is stopped when program is killed.

        Test Sequence:
            1. Request robot movement in a subprocess.
            2. Send kill signal.

        Expected Results:
            1. -
            2. Robot does not move after subprocess has terminated.
        """

        # 1. Start robot movement
        proc = subprocess.Popen(self._get_robot_move_command())

        # Wait until movement is detected
        self.assertTrue(self.robot_motion_observer.wait_motion_start())

        # 2. Send kill signal
        proc.send_signal(signal.SIGINT)

        # Wait until process has terminated.
        proc.wait()

        self.assertFalse(self.robot_motion_observer.is_robot_moving())


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_program_kill')
    rostest.rosrun('pilz_robot_programming', 'test_api_program_kill', TestAPIProgramKill)
