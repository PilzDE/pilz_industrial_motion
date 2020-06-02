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

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from rospy.rostime import Time

start_pose_test = PoseStamped(
    header=Header(
        seq=0,
        stamp=Time(
            secs=12345,
            nsecs=6789
        ),
        frame_id='world'
    ),
    pose=Pose(
        position=Point(
            x=7,
            y=0.0,
            z=0.0
        ),
        orientation=Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=-1.0
        )
    )
)
goal_pose = PoseStamped(
    header=Header(
        seq=0,
        stamp=Time(
            secs=123,
            nsecs=89
        ),
        frame_id='tcp'
    ),
    pose=Pose(
        position=Point(
            x=0.0,
            y=0.8,
            z=0.0
        ),
        orientation=Quaternion(
            x=0.0,
            y=1.0,
            z=0.0,
            w=0.0
        )
    )
)
