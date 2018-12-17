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
import threading
from pilz_robot_programming.robot import RobotMoveFailed


def waited_trigger(cv, func):
    with cv:
        cv.wait()
    func()


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
