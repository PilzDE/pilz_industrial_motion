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
import rospy

from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.integration_test_utils import MoveThread

from pilz_test_facility.test_facility_manager import TestFacilityManager
from pilz_test_facility.test_facility_sensors import TestFacilitySensors
from pilz_test_facility.op_modes import OperationMode

_DEFAULT_VEL_SCALE = 0.8


class TestMoveControl(unittest.TestCase):
    """Tests the features 'stop', 'pause' and 'resume' of the Python API."""

    _TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.3
    _REQUIRED_API_VERSION = "1"

    _HOME_POSE = [0, 0, 0, 0, 0, 0]
    _FIRST_POSE = [0, -0.52, 0, 0, 0, 0]
    _SECOND_POSE = [0, 0.52, 0, 0, 0, 0]

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_move_ctrl')

    def _move_robot_to_default_start_pose(self):
        rospy.loginfo("Test-SetUp: Move robot to default start pose: " + str(self._HOME_POSE) + "...")
        with self._test_facility:
            self._test_facility.ready_robot_for_motion_in(OperationMode.T1)
            self._robot.move(Ptp(goal=self._HOME_POSE))
        rospy.loginfo("Test-SetUp: FINISHED - Move robot to default start pose: " + str(self._HOME_POSE))

        self.assertTrue(self._sensors.is_robot_at_position(self._HOME_POSE),
                        msg="Robot not at default robot start pose: " + str(self._HOME_POSE))

    def setUp(self):
        rospy.loginfo("SetUp called...")

        self._robot = Robot(self._REQUIRED_API_VERSION)

        self._sensors = TestFacilitySensors()
        self._sensors.set_default_motion_detection_tolerance(self._TOLERANCE_FOR_MOTION_DETECTION_RAD)

        self._test_facility = TestFacilityManager()

        self._move_robot_to_default_start_pose()

    def tearDown(self):
        rospy.loginfo("TearDown called...")
        self._robot._release()
        self._robot = None

    def test_stop(self):
        """Tests stopping a robot motion and executing a new robot motion."""
        with self._test_facility:
            self._test_facility.ready_robot_for_motion_in(OperationMode.T1)

            move_thread = MoveThread(self._robot, Ptp(goal=self._FIRST_POSE, vel_scale=_DEFAULT_VEL_SCALE))
            move_thread.start_async_motion()
            self.assertTrue(self._sensors.wait_for_robot_motion(), msg="Robot should have started moving.")

            self._robot.stop()
            move_thread.wait_until_move_cmd_finished()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot is still moving")
            self.assertFalse(self._sensors.is_robot_at_position(self._FIRST_POSE),
                             msg="Due to the executed stop, the robot should have not reached the goal position.")

            self._robot.move(Ptp(goal=self._SECOND_POSE, vel_scale=_DEFAULT_VEL_SCALE))
            self.assertTrue(self._sensors.is_robot_at_position(self._SECOND_POSE),
                            msg="Robot did not reach goal position.")

    def test_pause_resume(self):
        """Tests pausing and resuming a robot motion."""
        with self._test_facility:
            self._test_facility.ready_robot_for_motion_in(OperationMode.T1)

            move_thread = MoveThread(self._robot, Ptp(goal=self._FIRST_POSE, vel_scale=_DEFAULT_VEL_SCALE))
            move_thread.start_async_motion()
            self.assertTrue(self._sensors.wait_for_robot_motion(), msg="Robot should have started moving")

            self._robot.pause()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should not move after call to pause()")
            self.assertFalse(self._sensors.is_robot_at_position(self._FIRST_POSE),
                             msg="Due to the call to pause(), the robot should have not reached the goal.")

            self._robot.resume()
            self.assertTrue(self._sensors.wait_for_robot_motion(), msg="Robot should have started moving")

            move_thread.wait_until_move_cmd_finished()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should have finished motion")
            self.assertTrue(self._sensors.is_robot_at_position(self._FIRST_POSE),
                            msg="Robot did not reach goal position.")

    def test_pause_stop(self):
        """Tests pausing and canceling a robot motion."""
        with self._test_facility:
            self._test_facility.ready_robot_for_motion_in(OperationMode.T1)

            move_thread = MoveThread(self._robot, Ptp(goal=self._FIRST_POSE, vel_scale=_DEFAULT_VEL_SCALE))
            move_thread.start_async_motion()
            self.assertTrue(self._sensors.wait_for_robot_motion(), msg="Robot should have started moving")

            self._robot.pause()
            self.assertTrue(self._sensors.wait_till_robot_stopped(), msg="Robot should stop after call to pause()")
            self.assertFalse(self._sensors.is_robot_at_position(self._FIRST_POSE),
                             msg="Due to the call to pause(), the robot should have not reached the goal.")

            self._robot.stop()
            move_thread.wait_until_move_cmd_finished()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should not move after call to stop()")
            self.assertFalse(self._sensors.is_robot_at_position(self._FIRST_POSE),
                             msg="Robot should have not reached goal, after call to pause() and stop()")

    def test_pause_between_moves(self):
        """Tests pausing the next motion when no motion is active."""
        first_move_thread = MoveThread(self._robot, Ptp(goal=self._FIRST_POSE, vel_scale=_DEFAULT_VEL_SCALE))
        second_move_thread = MoveThread(self._robot, Ptp(goal=self._HOME_POSE, vel_scale=_DEFAULT_VEL_SCALE))

        with self._test_facility:
            self._test_facility.ready_robot_for_motion_in(OperationMode.T1)

            first_move_thread.start_async_motion()
            self.assertTrue(self._sensors.wait_for_robot_motion(), msg="Robot should have started moving")

            first_move_thread.wait_until_move_cmd_finished()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should have finished motion")
            self.assertTrue(self._sensors.is_robot_at_position(self._FIRST_POSE),
                            msg="Robot did not reach goal position.")

            self._robot.pause()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should have not started moving")
            self.assertTrue(self._sensors.is_robot_at_position(self._FIRST_POSE),
                            msg="Robot not at expected position.")

            second_move_thread.start_async_motion()
            self.assertFalse(self._sensors.wait_for_robot_motion(),
                             msg="Robot should have not started moving while pause is active")
            self.assertTrue(self._sensors.is_robot_at_position(self._FIRST_POSE),
                            msg="Robot not at expected position.")

            self._robot.resume()
            second_move_thread.wait_until_move_cmd_finished()
            self.assertFalse(self._sensors.is_robot_moving(), msg="Robot should have not started moving")
            self.assertTrue(self._sensors.is_robot_at_position(self._HOME_POSE),
                            msg="Robot not at expected position.")


if __name__ == "__main__":
    unittest.main()
