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
from geometry_msgs.msg import PoseStamped


class MoveGroupCommanderMock:
    """Mock for testing."""
    def __init__(self):
        rospy.loginfo("Ctor of MoveGroupCommanderMock called")

    def get_end_effector_link(self):
        rospy.loginfo("get_end_effector_link called")
        return ["prbt_tcp"]

    def get_active_joints(self):
        rospy.loginfo("get_active_joints called")
        return ["joint1", "joint2", "joint3", "joint4", "joint5,", "joint6"]

    def get_current_joint_values(self):
        rospy.loginfo("get_current_joint_values called")
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def get_current_pose(self, target_link=""):
        rospy.loginfo("get_current_pose called")
        return PoseStamped()

class RobotCommanderMock:
    """Mock for testing."""
    def __init__(self):
        rospy.loginfo("Ctor of RobotCommanderMock called")

    def get_group_names(self):
        rospy.loginfo("get_group_names called")
        return ["manipulator"]

    def get_group(self, group_name):
        rospy.loginfo("get_group called")
        return MoveGroupCommanderMock()

    def get_planning_frame(self):
        rospy.loginfo("get_planning_frame() called")
        return "dummy_str"
