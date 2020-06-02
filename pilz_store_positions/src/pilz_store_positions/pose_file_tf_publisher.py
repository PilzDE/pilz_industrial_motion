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

import sys
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class PoseFileTFPublisher(object):
    """!
    Publishes TF poses from a given python file containing various poses on the /tf topic.
    """

    def __init__(self, file_path):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self._import_pose_list(file_path)

    def _import_pose_list(self, file_path):
        file_name_without_extension, path_to_file = self._get_name_and_path(file_path)
        sys.path.append(path_to_file)
        self.__pose_list_file = __import__(file_name_without_extension)
        sys.path.pop()

    @staticmethod
    def _get_name_and_path(file_path):
        last_slash_index = file_path.rfind("/")
        path_to_file = file_path[:last_slash_index]
        file_name = file_path[last_slash_index + 1:]
        file_name_without_extension = file_name.split(".")[0]
        return file_name_without_extension, path_to_file

    def publish_poses(self):
        for k, v in self.__pose_list_file.__dict__.items():
            try:
                t = TransformStamped(child_frame_id=k)
                t.transform.translation = v.pose.position
                t.transform.rotation = v.pose.orientation
                t.header.frame_id = v.header.frame_id
                t.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(t)
            except AttributeError:
                if not callable(v) and v.__class__ is not None and not k.startswith("_"):
                    rospy.logerr("%s is of unknown type: %s" % (k, v.__class__))
