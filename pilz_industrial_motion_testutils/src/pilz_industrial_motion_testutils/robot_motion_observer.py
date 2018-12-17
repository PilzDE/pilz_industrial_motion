#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright © 2018 Pilz GmbH & Co. KG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy
from moveit_commander import RobotCommander, MoveItCommanderException


class RobotMotionObserver(object):
    """Observes motions of the robot.

    The implementation is based on using the get_current_joint_values()) functionality of the RobotCommander().

    :param group_name: Name of the planning group, default value is 'manipulator'
    """

    _DEFAULT_WAIT_TIME_FOR_MOTION_DETECTION_SEC = 3.0
    _DEFAULT_TOLERANCE_FOR_MOTION_DETECTION_RAD = 0.01
    _DEFAULT_SLEEP_INTERVAL_SEC = 0.01
    _DEFAULT_GROUP_NAME = "manipulator"

    def __init__(self, group_name=_DEFAULT_GROUP_NAME):
        self._robot_commander = RobotCommander()
        self._group_name = group_name

    @staticmethod
    def _detect_motion(joint_values_a, joint_values_b, tolerance=_DEFAULT_TOLERANCE_FOR_MOTION_DETECTION_RAD):
        """TRUE if a significant motion was detected, otherwise FALSE."""
        return not numpy.allclose(joint_values_a, joint_values_b,
                                  atol=tolerance)

    def _get_current_joint_states(self):
        """Returns the current joint state values of the robot.
        :raises RobotCurrentStateError if given planning group does not exist.
        """
        try:
            return self._robot_commander.get_group(self._group_name).get_current_joint_values()
        except MoveItCommanderException as e:
            rospy.logerr(e.message)
            raise RobotCurrentStateError(e.message)

    def is_robot_moving(self,
                        sleep_interval=_DEFAULT_SLEEP_INTERVAL_SEC,
                        move_tolerance=_DEFAULT_TOLERANCE_FOR_MOTION_DETECTION_RAD):
        """TRUE if the robot is currently moving, otherwise FLASE."""
        start_position = self._get_current_joint_states()
        rospy.sleep(sleep_interval)
        return RobotMotionObserver._detect_motion(start_position, self._get_current_joint_states(), move_tolerance)

    def wait_motion_start(self,
                          wait_time_out=_DEFAULT_WAIT_TIME_FOR_MOTION_DETECTION_SEC,
                          move_tolerance=_DEFAULT_TOLERANCE_FOR_MOTION_DETECTION_RAD,
                          sleep_interval=_DEFAULT_SLEEP_INTERVAL_SEC):
        """TRUE if the motion started in the given time interval, otherwise FALSE."""

        old_joint_values = self._get_current_joint_states()
        rospy.loginfo("Start joint values: " + str(old_joint_values))

        motion_started = False
        start_time = rospy.get_time()
        rospy.loginfo("Wait until motion starts...")

        while True:
            rospy.sleep(sleep_interval)
            curr_joint_values = self._get_current_joint_states()

            if RobotMotionObserver._detect_motion(curr_joint_values, old_joint_values, move_tolerance):
                motion_started = True
                rospy.loginfo("Changed joint values detected: " + str(curr_joint_values))
                rospy.loginfo("Motion started.")
                break

            if (rospy.get_time() - start_time > wait_time_out):
                rospy.loginfo("Reached timeout when waiting for motion start.")
                break

        return motion_started

    def wait_motion_stop(self,
                         wait_time_out=_DEFAULT_WAIT_TIME_FOR_MOTION_DETECTION_SEC,
                         move_tolerance=_DEFAULT_TOLERANCE_FOR_MOTION_DETECTION_RAD,
                         sleep_interval=_DEFAULT_SLEEP_INTERVAL_SEC):
        """TRUE if the motion stopped in the given time interval, otherwise FALSE."""

        motion_stopped = False
        start_time = rospy.get_time()
        rospy.loginfo("Wait until motion stops...")

        while True:

            if not self.is_robot_moving(sleep_interval, move_tolerance):
                motion_stopped = True
                rospy.loginfo("Motion stopped.")
                break

            if (rospy.get_time() - start_time > wait_time_out):
                rospy.loginfo("Reached timeout when waiting for motion stop.")
                break

        return motion_stopped
