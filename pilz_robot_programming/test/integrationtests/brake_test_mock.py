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

from prbt_hardware_support.srv import IsBrakeTestRequired, IsBrakeTestRequiredResponse

BRAKE_TEST_REQUIRED_SERVICE_NAME = "/prbt/is_brake_test_required"

class BrakeTestMock(Thread):
    """
    Mock of Brake Test Interfaces.
    """

    def __init__(self):
        """Default Constructor"""
        super(BrakeTestMock, self).__init__()
        rospy.loginfo("Mocking Brake Test")
        self._is_braketest_required_service_mock = None
        self._is_braketest_required = False
        self._running = True

    def _is_brake_test_required_server_handler(self, req):
        res = IsBrakeTestRequiredResponse()
        res.result = self._is_braketest_required
        return res

    def advertise_service(self):
        """Method to manually advertise the service, when necessary"""
        self._is_braketest_required_service_mock = rospy.Service(
            name=BRAKE_TEST_REQUIRED_SERVICE_NAME,
            service_class=IsBrakeTestRequired,
            handler=self._is_brake_test_required_server_handler
        )

    def run(self):
        """Overriding thread method to be called with Thread.start()"""
        self._running = True
        while self._running and not rospy.is_shutdown():
            rospy.sleep(.1)

    def stop(self):
        """Method to stop the sleep loop in run()"""
        if self._is_braketest_required_service_mock:
            self._is_braketest_required_service_mock.shutdown()
        self._running = False

    def set_is_braketest_required(self, state):
        """Set the status that the service should return.

        :param state: status that the service should return
        """
        self._is_braketest_required = state