# Store poses during teaching

## Preparation

To prepare for saving position you have to specify a file.
Open a terminal and enter:

```
roslaunch teach_positions.launch [_file_path := "/path/to/file.py"]
```

If no path is given the file is created in the current directory.
If the file already exists, it is overwritten.

## Teaching

Move the robot to the desired position.

Using the following service calls the current positions/joints can be saved to the file:

```
rosservice call save_position [name := <pick_pose>]
```

```
rosservice call save_joints [name := <pick_pose>]
```

If the name-argument is not specified, the teached positions/joints are named according to the following rule:


`positionX`, Where `X` is an incremented counter. (`jointX` in case of the `save_joints` service.)

Repeat this procedure for every teach position.


# Using stored positions in a python script

The generated file can be included into your pilz_robot_programming script as follows:

```
#!/usr/bin/env python
from pilz_robot_programming import *
import points as pts # Replace points by the filename you have chosen for the positions
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


# Display Stored Points
Stored points can be published as target_frames. 
Those target frames can for example be displayed by Rviz.

To publish the points run:
```
rosrun pilz_store_positions pose_visualisation_node _file_path:="/path/to/file.py"
```
