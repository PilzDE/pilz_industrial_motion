#!/bin/bash
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

# move to start pose by ptp
# 3. Robot moves to start pose via PTP motion.
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.2 ptp joint 1.570 0.029 -2.03 1.63 2.06 -1.57
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.2 ptp pose 0.0 0.28 0.6 0 0 0


# test circ
# 4. Robot moves 1/4 circle from the start pose.
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.1 circ pose 0.28 0 0.6 0 1.0 0 center 0 0 0.6
# 5. Robot moves 1/4 circle from the current pose.
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.1 circ pose 0 -0.28 0.6 0 0 0 center 0 0 0.6
# 6. Robot moves back 1/4 circle from the current pose.
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.1 circ pose 0.28 0 0.6 0 1.0 0 center 0 0 0.6
# 7. Robot moves back 1/4 circle from the current pose and return to the start pose.
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.1 circ pose 0 0.28 0.6 0 0 0 center 0 0 0.6
# 8. Robot moves 5/12 circle (150 deg) from the start pose.
rosrun pilz_robot_programming movecmd.py -s 0.1 -a 0.1 circ pose 0.14 -0.24249 0.6 0 1.57 0 interim 0.28 0 0.6
# 9. Robot moves to start pose.
rosrun pilz_robot_programming movecmd.py -s 0.1 -a 0.1 circ pose 0 0.28 0.6 0 0 0 center 0 0 0.6
# 10. Robot moves 5/8 circle from the start pose.
rosrun pilz_robot_programming movecmd.py -s 0.1 -a 0.1 circ pose -0.19799 -0.19799 0.6 0 0 0 interim 0.28 0 0.6


# 11. Robot moves to start pose via PTP motion (0 0.05 0.6 0 0 0)
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.2 ptp joint 1.5707 -0.823 -2.442 0 1.618 -1.57
rosrun pilz_robot_programming movecmd.py -s 0.3 -a 0.2 ptp pose 0 0.05 0.6 0 0 0
# circle: center 0 0.2 0.6, radius 0.15
# 12. Robot moves along the circle with 1.99 PI.
rosrun pilz_robot_programming movecmd.py -s 0.05 -a 0.05 circ pose -0.01480197 0.0573415225 0.6 0 0 0 interim 0.15 0.20 0.6
