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
import threading
from pilz_robot_programming.robot import RobotMoveFailed


def waited_trigger(cv, func):
    with cv:
        cv.wait()
    func()


def detect_motion(joint_values_a, joint_values_b, tolerance):
    """TRUE if a significant motion was detected, otherwise FALSE."""
    return not numpy.allclose(joint_values_a, joint_values_b,
                              atol=tolerance)


def is_robot_moving(robot, time=0.1, tolerance=0.01):
    start_position = robot.get_current_joint_states()
    rospy.sleep(time)
    return detect_motion(start_position, robot.get_current_joint_states(), tolerance)


def wait_cmd_start(robot, sleep_time, move_tolerance, group_name="manipulator"):
    """Wait till movement starts. """
    movement_started_flag = False
    old_joint_values = robot.get_current_joint_states(group_name)
    rospy.loginfo("Start joint values: " + str(old_joint_values))
    rospy.loginfo("Wait until motion started...")
    while not movement_started_flag:
        rospy.sleep(sleep_time)
        # Check current joint values
        curr_joint_values = robot.get_current_joint_states(group_name)
        if detect_motion(curr_joint_values, old_joint_values, move_tolerance):
            movement_started_flag = True
            rospy.loginfo("Changed joint values detected: " + str(curr_joint_values))
            rospy.loginfo("Motion started.")


def wait_cmd_stop(robot, wait_time_out, move_tolerance, sleep_interval=0.01):
    """Wait till movement stops. """
    old_joint_values = robot.get_current_joint_states()

    rospy.loginfo("Start joint values: " + str(old_joint_values))
    rospy.loginfo("Wait until motion stops...")

    wait_time = 0

    while wait_time < wait_time_out:
        rospy.sleep(sleep_interval)
        wait_time += sleep_interval
        curr_joint_values = robot.get_current_joint_states()
        if not detect_motion(curr_joint_values, old_joint_values, move_tolerance):
            rospy.loginfo("Motion stopped.")
            return True
        else:
            old_joint_values = curr_joint_values
            rospy.loginfo("Changed joint values detected: " + str(curr_joint_values))

    rospy.loginfo("Motion did not stop in " + str(wait_time) + " seconds")
    return False


class MoveThread(threading.Thread):
    def __init__(self, robot, cmd):
        threading.Thread.__init__(self)
        self._robot = robot
        self._cmd = cmd
        self.exception_thrown = False

    def run(self):
        rospy.logdebug("Start motion...")
        try:
            self._robot.move(self._cmd)
        except RobotMoveFailed:
            rospy.logdebug("Caught expected exception.")
            self.exception_thrown = True
