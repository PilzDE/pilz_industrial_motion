from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

start_pose_test = Pose(
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
