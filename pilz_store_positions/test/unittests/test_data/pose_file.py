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
