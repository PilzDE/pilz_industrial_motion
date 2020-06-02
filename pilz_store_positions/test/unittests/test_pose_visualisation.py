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
import tf2_ros
import rospkg
import pytest
from contextlib import contextmanager
from pilz_store_positions import PoseFileTFPublisher
from mock import Mock, call
from geometry_msgs.msg import TransformStamped, Transform, Point, Quaternion
from std_msgs.msg import Header


PKG = 'pilz_store_positions'


@contextmanager
def does_not_raise():
    yield


pose_file_output = (call(TransformStamped(header=Header(stamp=rospy.Time(100, 101), frame_id="tcp"),
                                          child_frame_id="goal_pose",
                                          transform=Transform(translation=Point(y=.8),
                                                              rotation=Quaternion(y=1.0)))),
                    call(TransformStamped(header=Header(stamp=rospy.Time(100, 101), frame_id="world"),
                                          child_frame_id="start_pose_test",
                                          transform=Transform(translation=Point(x=7.0),
                                                              rotation=Quaternion(w=-1.0)))),
                    )
unknown_type_err = (call("start_pose_test is of unknown type: <class 'geometry_msgs.msg._Pose.Pose'>"),)


@pytest.mark.parametrize("test_file,            expected_exception,         tf_calls,           ros_errs",
                         [("pose_file.py",      does_not_raise(),           pose_file_output,   ()),
                          ("missing_type_file", pytest.raises(NameError),   (),                 ()),
                          ("unknown_type",      does_not_raise(),           (),                 unknown_type_err),
                          ("yaml_file",         pytest.raises(ImportError), (),                 ()),
                          ("non_existent_file", pytest.raises(ImportError), (),                 ())])
def test_visualisation(test_file, expected_exception, tf_calls, ros_errs, monkeypatch):
    """
    serializes a ros msg, read back and compare with original message
    """
    __mock_dependencys(monkeypatch)
    publish_mock, err_mock = __mock_side_effects(monkeypatch)
    _test_data_dir = rospkg.RosPack().get_path("pilz_store_positions") + "/test/unittests/test_data/"

    with expected_exception:
        pb = PoseFileTFPublisher(_test_data_dir + test_file)
        pb.publish_poses()
        publish_mock.assert_has_calls(tf_calls)
        err_mock.assert_has_calls(ros_errs)


def __mock_side_effects(monkeypatch):
    publish_mock = Mock()
    monkeypatch.setattr(tf2_ros.TransformBroadcaster, "sendTransform", publish_mock)
    err_mock = Mock()
    monkeypatch.setattr(rospy, "logerr", err_mock)
    return publish_mock, err_mock


def __mock_dependencys(monkeypatch):
    monkeypatch.setattr(rospy.Time, "now", Mock(return_value=rospy.Time(100, 101)))
