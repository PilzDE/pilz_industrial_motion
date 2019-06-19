<!--
Copyright (c) 2019 Pilz GmbH & Co. KG

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

# Acceptance Test for brake test using Python-API and real robot
These acceptance tests check that the real robot system is able perform a brake test when triggered from the python API.

## Prerequisites
  - Properly connect and startup the robot. Make sure a emergency stop is within reach.
  - Make sure there is no brake the required
  - Make sure you can emulate the brake test required flag

## Starting the acceptance tests
- Bringup can: `sudo ip link set can0 up type can bitrate 1000000`
- Run `roslaunch prbt_moveit_config moveit_planning_execution.launch sim:=False pipeline:=pilz_command_planner`
- Run `roslaunch prbt_hardware_support modbus_brake_test_announcer_node.launch safety_hw:=pnoz`
- Run `roslaunch prbt_hardware_support brake_test_executor_node.launch`
- Run `rosrun pilz_robot_programming acceptance_test_brake_test.py`

## Test Precedure
- Start test as described
- Emulate the brake test required flag (i.e. *pushing the button*)
- Observe results

## Expected Result
- The programming is repeatedly checking the brake test required flag and is printing, that no brake test is required
- If you push the *brake-test-required-emulation-button*: Brake test requirement is detected
- Brake test is preformed
- Brake test is **successful** (i.e. no exception is thrown)

