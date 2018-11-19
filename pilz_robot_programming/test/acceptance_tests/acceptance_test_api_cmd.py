#!/usr/bin/env python
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

from geometry_msgs.msg import Point
from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.acceptance_test_utils import _askPermission, _askSuccess

PTP_VEL_PICK = 0.1


def start_program():
    print("Executing " + __file__)

    robot = Robot('0')
    _test_ptp_pos(robot)
    _test_lin_pos(robot)
    _test_seq_pos(robot)
    _test_circ_pos(robot)
    _test_blend_pos(robot)


def _test_ptp_pos(robot):
    """Tests a ptp motion.

        Test Sequence:
          1. Move to start position.
          2. Move away from the singularity.
          3. Move to the upper "pick" position

        Expected Results:
          1. Robot moves to start position.
          2. Robot moves away from the singularity.
          3. Robot moves to the upper "pick" position.
    """
    if _askPermission(_test_ptp_pos.__name__) == 0:
        return
    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))
    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))

    _askSuccess(_test_ptp_pos.__name__,
                'The robot should have move to [0,0,0,0,0,0] and after that to the next position.')


def _test_lin_pos(robot):
    """Test a lin motion

        Test Sequence:
          1. Move 15cm linear down.
          2. Move 15cm linear up.

        Expected Results:
          1. Robot moves 15cm linear down.
          2. Robot moves 15cm linear up.
    """
    if _askPermission(_test_lin_pos.__name__) == 0:
        return
    robot.move(Lin(goal=Pose(position=Point(0.0, 0.0, -0.15)), relative=True, vel_scale=PTP_VEL_PICK))
    robot.move(Lin(goal=Pose(position=Point(0.0, 0.0, 0.15)), relative=True, vel_scale=PTP_VEL_PICK))

    _askSuccess(_test_lin_pos.__name__, 'The robot tcp should move linear 15cm down and then linear 15cm up.')


def _test_seq_pos(robot):
    """Tests a motion sequence.

        Test Sequence:
            1. Execute sequence consisting of the following motions:
                a. Move to start position.
                b. Move to the upper "pick" position.
                c. Move 15cm linear down.
                d. Move 15cm linear up.

        Expected Results:
            1. Robot performes sequence of the following motions stopping in between them:
                a. Robot moves to start position.
                b. Robot moves to the upper "pick" position.
                c. Robot moves 15cm linear down.
                d. Robot moves 15cm linear up.
    """
    if _askPermission(_test_seq_pos.__name__) == 0:
        return

    x_pick = 0
    y_pick = 0.25
    z_pick_low = 0.25
    z_pick_high = 0.4

    rot = Quaternion(0.0, 1.0, 0.0, 0.0)
    p_pick_high = Pose(position=Point(x_pick, y_pick, z_pick_high), orientation=rot)
    p_pick_low = Pose(position=Point(x_pick,  y_pick, z_pick_low),  orientation=rot)

    seq_l = Sequence()
    seq_l.append(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    seq_l.append(Ptp(goal=p_pick_high))
    seq_l.append(Lin(goal=p_pick_low))
    seq_l.append(Lin(goal=p_pick_high))

    robot.move(seq_l)

    _askSuccess(_test_seq_pos.__name__, 'The robot should have moved to [0,0,0,0,0,0] and back to the upper pick'
                                        + 'position. After that The robot tcp should have moved linear 15cm down and'
                                        + 'then linear 15cm up.')


def _test_circ_pos(robot):
    """Test a circ motion

        Test Sequence:
          1. Move half a circle.
          2. Move half a circle back to origin.

        Expected Results:
          1. Robot moves half a circle.
          2. Robot moves half a circle back to origin.
    """
    if _askPermission(_test_circ_pos.__name__) == 0:
        return

    robot.move(Circ(goal=Pose(position=Point(-0.460, 0, 0.19),
                    orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK,
                    interim=Point(-0.355, -0.105, 0.19)))

    robot.move(Circ(goal=Pose(position=Point(-0.460, -0.21, 0.19),
                    orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK,
                    interim=Point(-0.565, -0.105, 0.19)))

    _askSuccess(_test_circ_pos.__name__, 'The robot should have moved in a half circle stop and move in half\
                                        a circle to its original position')


def _test_blend_pos(robot):
    """ Test a motion blending

        Test Sequence:
            1. Generate square movement by blending 4 lin.

        Expected Results:
            1. Robot moves in a square without intermediate stops.
    """
    if _askPermission(_test_blend_pos.__name__) == 0:
        return

    seq_l = Sequence()
    seq_l.append(Lin(goal=Pose(position=Point(-0.460, 0, 0.19),
                       orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK), blend_radius=0.05)

    seq_l.append(Lin(goal=Pose(position=Point(-0.3, 0, 0.19),
                       orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK), blend_radius=0.05)

    seq_l.append(Lin(goal=Pose(position=Point(-0.3, -0.21, 0.19),
                       orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK), blend_radius=0.05)

    seq_l.append(Lin(goal=Pose(position=Point(-0.46, -0.21, 0.19),
                       orientation=from_euler(0, -3.14, -0.25)), vel_scale=PTP_VEL_PICK), blend_radius=0.0)

    robot.move(seq_l)

    _askSuccess(_test_blend_pos.__name__, 'The robot should move in a square back to its original position.\
                                         During the motion the robot does not stop.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('api_acceptance_tests_node', anonymous=True)

    start_program()
