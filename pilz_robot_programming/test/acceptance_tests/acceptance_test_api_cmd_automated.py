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

import time

from geometry_msgs.msg import Point

from pilz_robot_programming.robot import *
from pilz_robot_programming.commands import *
from pilz_industrial_motion_testutils.acceptance_test_utils import _askPermission, _askSuccess

from pilz_robot_tfc_api.modbus_tfc_api import ModbusTfcAPI
from pilz_robot_tfc_api.pilz_modbus_client import PilzModbusClient
from pilz_robot_tfc_api.op_modes import OperationMode

PTP_VEL_PICK = 0.1

_REQUIRED_API_VERSION = "1"


def start_program():
    print("Executing " + __file__)

    robot = Robot(_REQUIRED_API_VERSION)
    _test_lin_pos(robot)


def _test_lin_pos(robot):
    """Test a lin motion

        Test Sequence:
          1. Move to start position using Ptp (upper "pick" position)
          2. Move 15cm linear down.
          3. Move 15cm linear up.

        Expected Results:
          1. Robot moves to the upper "pick" position
          2. Robot moves 15cm linear down.
          3. Robot moves 15cm linear up.
    """
    # if _askPermission(_test_lin_pos.__name__) == 0:
    #    return
    tfc = ModbusTfcAPI(PilzModbusClient())
    tfc.open()
    tfc.disable_emergency()
    tfc.acknowledge_ready_signal()
    tfc.choose_operation_mode(OperationMode.Auto)
    tfc.acknowledge_ready_signal()
    tfc.activate_enabling()

    robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))
    robot.move(Ptp(goal=[0, -0.78, 0.78, 0, 1.56, 0]))

    robot.move(Ptp(goal=Pose(position=Point(-0.46, -0.21, 0.19), orientation=from_euler(0, -3.14, -0.25))))
    robot.move(Lin(goal=Pose(position=Point(0.0, 0.0, -0.15)), relative=True, vel_scale=PTP_VEL_PICK))
    robot.move(Lin(goal=Pose(position=Point(0.0, 0.0, 0.15)), relative=True, vel_scale=PTP_VEL_PICK))

    tfc.deactivate_enabling()

    tfc.close()

    # _askSuccess(_test_lin_pos.__name__, 'The robot should have moved to the upper pick position. Afterwards the tcp'
    #                                    + ' should have moved linear 15cm down and then linear 15cm up.')


if __name__ == "__main__":
    # Init a ros node
    rospy.init_node('api_acceptance_tests_node', anonymous=True)

    start_program()
