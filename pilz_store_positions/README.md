# Store poses during teaching

After moving the robot to the desired position, you can store the current position to file.

To save the current robot pose as ros_msg to a file use `rosrun pilz_store_postions store_current_pose`.
The generated file is stored at your current directory as `points.py` and can be included into your pilz_robot_programming script as follows:

```
#!/usr/bin/env python
from pilz_robot_programming import *
import points as pts
import rospy

__REQUIRED_API_VERSION__ = "1"    # API version
__ROBOT_VELOCITY__ = 0.5          # velocity of the robot

# main program
def start_program():

    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=pts.pick_pose, vel_scale=__ROBOT_VELOCITY__))

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
```

while the generated points.py looks similar as:
```
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

pick_pose = Pose(
    position = Point(
      x = 0.07,
      y = 0.40,
      z = 0.50
    ),
    orientation = Quaternion(
      x = 1.0,
      y = 0.0,
      z = 0.0,
      w = 0.0
    )
)
```

### Display Stored Points
Stored points can be published as target_frames. 
Those target frames can for example be displayed by Rviz.

To publish the points run:
```
rosrun pilz_store_positions pose_visualisation_node _file_path:="/absolute/path/to/points.py"
```
