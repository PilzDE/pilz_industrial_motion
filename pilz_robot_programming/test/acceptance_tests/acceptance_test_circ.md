<!--
Copyright (c) 2018 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
-->

# Acceptance Test CIRC Motion using movecmd.py on the real robot
This test checks that the real robot system is able to perform a CIRC Motion to a goal state given by the user. The test is performed using the moveit motion planning plugin. Note that before you can apply the CIRC command the robot has
to be moved out of singularities.

## Prerequisites
  - Properly connect and startup the robot. Make sure a emergency stop is within reach.

## Test Sequence:
  1. Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
  2. Run `rosrun pilz_trajectory_generation acceptance_test_circ.sh`

## Expected Results:
  1. Can should be visible with `ifconfig` displayed as can0
  2. A -click- indicates the enabling of the drives.
  3. Robot moves to start pose via PTP motion.
  4. Robot moves 1/4 circle from the start pose.
  5. Robot moves 1/4 circle from the current pose.
  6. Robot moves back 1/4 circle from the current pose.
  7. Robot moves back 1/4 circle from the current pose and return to the start pose.
  8. Robot moves 5/12 circle (150 deg) from the start pose.
  9. Robot moves to start pose.
  10. Robot moves 5/8 circle from the start pose.
  11. Robot moves to new start pose via PTP motion.
  12. Robot moves along the circle with 1.99 PI.
