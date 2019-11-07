#!/usr/bin/env python

# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import roslaunch
import rospkg
import rospy

PACKAGE_NAME = 'pilz_extensions'

## Test the pilz_extensions demo
#
# This test exists for code coverage of pilz_extensions_demo_node.cpp
class IntegrationtestPilzExtensionsDemo(unittest.TestCase):

    ##  Test Sequence:
    #       1. Start demo launch file and spin until shutdown signal is received
    #
    #   Expected Results:
    #       1. -
    def runTest(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path(PACKAGE_NAME)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(uuid, [path + "/launch/pilz_extensions_demo.launch"])
        launch.start()

        try:
            launch.spin()
        finally:
            launch.shutdown()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PACKAGE_NAME, 'integrationtest_pilz_extensions_demo',
                   IntegrationtestPilzExtensionsDemo)
