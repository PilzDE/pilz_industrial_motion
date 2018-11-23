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
import subprocess
import signal
from rospkg import RosPack
from moveit_commander import RobotCommander
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_industrial_motion_testutils.integration_test_utils import detect_motion
from pilz_robot_programming.commands import *

_TEST_DATA_FILE_NAME = RosPack().get_path("pilz_industrial_motion_testutils") + "/test_data/testdata.xml"
_GROUP_NAME = "manipulator"


class TestAPIProgramKill(unittest.TestCase):
    """
    Test the API behaviour when the python program is killed.
    """

    _SLEEP_TIME_S = 0.01
    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01
    _PTP_TEST_NAME = "PTPJointValid"

    def setUp(self):
        rospy.loginfo("SetUp called...")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.robot_commander = RobotCommander()

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        if hasattr(self, 'test_data'):
            del self.test_data

    def get_current_joint_values(self, group_name="manipulator"):
        return self.robot_commander.get_group(group_name).get_current_joint_values()

    def detect_motion(self, sleep_time=_SLEEP_TIME_S, move_tolerance=_TOLERANCE_FOR_MOTION_DETECTION_RAD):
        """Check if robot is currently moving."""
        old_joint_values = self.get_current_joint_values()
        rospy.sleep(sleep_time)
        new_joint_values = self.get_current_joint_values()
        return detect_motion(new_joint_values, old_joint_values, move_tolerance)

    def wait_cmd_start(self, sleep_time=_SLEEP_TIME_S, move_tolerance=_TOLERANCE_FOR_MOTION_DETECTION_RAD):
        """Wait till movement starts. """
        movement_started_flag = False
        old_joint_values = self.get_current_joint_values()
        rospy.loginfo("Start joint values: " + str(old_joint_values))
        rospy.loginfo("Wait until motion started...")
        while not movement_started_flag:
            rospy.sleep(sleep_time)
            # Check current joint values
            curr_joint_values = self.get_current_joint_values()
            if detect_motion(curr_joint_values, old_joint_values, move_tolerance):
                movement_started_flag = True
                rospy.loginfo("Changed joint values detected: " + str(curr_joint_values))
                rospy.loginfo("Motion started.")

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
        self.wait_cmd_start()

        # 2. Send kill signal
        proc.send_signal(signal.SIGINT)

        # Wait until process has terminated.
        proc.wait()

        self.assertFalse(self.detect_motion())


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_program_kill')
    rostest.rosrun('pilz_robot_programming', 'test_api_program_kill', TestAPIProgramKill)
