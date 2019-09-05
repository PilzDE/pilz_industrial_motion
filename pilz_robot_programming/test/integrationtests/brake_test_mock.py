#!/usr/bin/env python
# Copyright (c) 2019 Pilz GmbH & Co. KG
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

from threading import Thread

import rospy

from prbt_hardware_support.srv import \
    BrakeTest, \
    BrakeTestResponse, \
    IsBrakeTestRequired, \
    IsBrakeTestRequiredResponse

from prbt_hardware_support.msg import BrakeTestErrorCodes
from prbt_hardware_support.srv import IsBrakeTestRequiredResponse

BRAKE_TEST_REQUIRED_SERVICE_NAME = "/prbt/brake_test_required"
BRAKE_TEST_EXECUTE_SERVICE_NAME = "/prbt/execute_braketest"


class BrakeTestMock(Thread):
    """
    Mock of Brake Test Interfaces.
    """

    def __init__(self):
        """Default Constructor"""
        super(BrakeTestMock, self).__init__()
        rospy.loginfo("Mocking Brake Test")
        self._is_brake_test_required_service_mock = None
        self._brake_test_execute_service_mock = None
        self._is_brake_test_required = IsBrakeTestRequiredResponse(result=IsBrakeTestRequiredResponse.NOT_REQUIRED)
        self._brake_test_execute_duration_s = .1
        self._brake_test_execute_result = BrakeTestErrorCodes.STATUS_SUCCESS
        self._brake_test_execute_msg = "Test Message"
        self._running = True

    def _is_brake_test_required_server_handler(self, _):
        return self._is_brake_test_required

    def _brake_test_execute_server_handler(self, _):
        # Sleeping ot simulate execution of brake test
        rospy.sleep(self._brake_test_execute_duration_s)
        res = BrakeTestResponse()
        res.success = self._brake_test_execute_result == BrakeTestErrorCodes.STATUS_SUCCESS
        res.error_code.value = self._brake_test_execute_result
        res.error_msg = self._brake_test_execute_msg
        return res

    def advertise_brake_test_required_service(self):
        """Method to manually advertise brake_test_required the service, when necessary"""
        self._is_brake_test_required_service_mock = rospy.Service(
            name=BRAKE_TEST_REQUIRED_SERVICE_NAME,
            service_class=IsBrakeTestRequired,
            handler=self._is_brake_test_required_server_handler
        )

    def advertise_brake_test_execute_service(self):
        """Method to manually advertise brake_test_execute the service, when necessary"""
        self._brake_test_execute_service_mock = rospy.Service(
            name=BRAKE_TEST_EXECUTE_SERVICE_NAME,
            service_class=BrakeTest,
            handler=self._brake_test_execute_server_handler
        )

    def run(self):
        """Overriding thread method to be called with Thread.start()"""
        self._running = True
        while self._running and not rospy.is_shutdown():
            rospy.sleep(.1)

    def stop(self):
        """Method to stop the sleep loop in run()"""
        if self._is_brake_test_required_service_mock is not None:
            self._is_brake_test_required_service_mock.shutdown()
        if self._brake_test_execute_service_mock is not None:
            self._brake_test_execute_service_mock.shutdown()
        self._running = False

    def set_is_brake_test_required_state(self, state):
        """Set the status that the service should return.

        :param state: status that the service should return
        """
        self._is_brake_test_required = IsBrakeTestRequiredResponse(result=state)

    def set_brake_test_execute_duration_s(self, duration):
        """Set the duration of the simulated brake test

        :param duration: desired duration in seconds
        """
        self._brake_test_execute_duration_s = duration

    def set_brake_test_execute_result(self, result):
        """Set the result that the simulated brake test should return

        :param result: desired result
        """
        self._brake_test_execute_result = result
