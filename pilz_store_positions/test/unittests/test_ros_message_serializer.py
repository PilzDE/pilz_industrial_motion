#!/usr/bin/env python

# Copyright (c) 2020 Pilz GmbH & Co. KG
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
import pytest
from pilz_store_positions import RosMessageSerializer


PKG = 'pilz_store_positions'


def single_pose():
    from geometry_msgs.msg import PoseStamped

    start_pose = PoseStamped()
    start_pose.header.stamp = rospy.Time(12345, 6789)
    start_pose.header.frame_id = "world"
    start_pose.pose.position.x = 7
    start_pose.pose.orientation.w = -1.0
    return {"start_pose_test": start_pose}


def multiple_poses():
    result = single_pose()
    from geometry_msgs.msg import PoseStamped

    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time(123, 89)
    goal_pose.header.frame_id = "tcp"
    goal_pose.pose.position.y = 0.8
    goal_pose.pose.orientation.y = 1.0

    result["goal_pose"] = goal_pose
    return result


@pytest.mark.parametrize("test_input", [{"hallo": "welt"},
                                        {},
                                        {"e": None},
                                        single_pose(),
                                        multiple_poses()])
def test_writeback(test_input, tmpdir, monkeypatch):
    """
    serializes a ros msg, read back and compare with original message
    """

    # save Pose with unique name
    module_name = tmpdir.basename
    RosMessageSerializer().write_messages_to_file(test_input, str(tmpdir.join(module_name + ".py")))

    # load module
    monkeypatch.syspath_prepend(str(tmpdir))
    readback = __import__(module_name)

    # compare all variables
    for k in test_input.keys():
        assert test_input[k] == getattr(readback, k), "Could not read back pose"
