#! /usr/bin/python

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

from prbt_hardware_support.msg import BrakeTestErrorCodes


class RobotVersionError(Exception):
    pass


class RobotMultiInstancesError(Exception):
    pass


class RobotMoveAlreadyRunningError(Exception):
    pass


class RobotMoveFailed(Exception):
    pass


class RobotUnknownCommandType(Exception):
    pass


class RobotCurrentStateError(Exception):
    pass


class RobotBrakeTestException(Exception):
    def __init__(self, result, message):
        _message = "{0}:{1}, msg: {2} ".format(
            result,
            self._result_nr_to_description(result),
            message)
        super(RobotBrakeTestException, self).__init__(_message)

    def _result_nr_to_description(self, result):
        for description in filter(str.isupper, BrakeTestErrorCodes.__dict__.keys()):
            if result.value == eval("BrakeTestErrorCodes." + description):
                return description
