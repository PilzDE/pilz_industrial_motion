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

import unittest
from rospkg import RosPack
import numpy as np
import tf
import tf_conversions.posemath as pm
from geometry_msgs.msg import Point


from pilz_robot_programming.robot import *
from pilz_industrial_motion_testutils.xml_testdata_loader import *
from pilz_robot_programming.commands import *

_TEST_DATA_FILE_NAME = RosPack().get_path("pilz_industrial_motion_testutils") + "/test_data/testdata.xml"
PLANNING_GROUP_NAME = "manipulator"
TARGET_LINK_NAME = "prbt_tcp"
API_VERSION = "1"

EXP_VEL_SCALE = 0.7
EXP_ACC_SCALE = 0.5

DBL_EPSILON = 1e-5

TF_POSITION_COMPARE_DELTA = 1e-3
TF_QUATERNION_COMPARE_DELTA = 1e-4

AXIS_SEQUENCE = "rzyz"
EULER_SAMPLE_NUM = 10
EULER_SAMPLES_RELATIVE_ROTATION = \
                [[a, b, c] \
                 for a in np.linspace(-pi, pi, EULER_SAMPLE_NUM) \
                 for b in np.linspace(-pi, pi, EULER_SAMPLE_NUM) \
                 for c in np.linspace(-pi, pi, EULER_SAMPLE_NUM)]


class TestAPICmdConversion(unittest.TestCase):
    """ Test the _cmd_to_request function of commands"""

    def _analyze_request_general(self, exp_move_group_name, exp_planner_id, exp_vel_scale, exp_acc_scale, act_request):
        self.assertEquals(exp_planner_id, act_request.planner_id)
        self.assertEquals(exp_move_group_name, act_request.group_name)
        self.assertEquals(exp_vel_scale, act_request.max_velocity_scaling_factor)
        self.assertEquals(exp_acc_scale, act_request.max_acceleration_scaling_factor)

    def _analyze_pose(self, exp_pose, req_pose, position_delta=None, quat_delta=DBL_EPSILON):
        self.assertAlmostEqual(exp_pose.position.x, req_pose.position.x,
                               delta=position_delta)
        self.assertAlmostEqual(exp_pose.position.y, req_pose.position.y,
                               delta=position_delta)
        self.assertAlmostEqual(exp_pose.position.z, req_pose.position.z,
                               delta=position_delta)

        exp_ori = np.asarray([exp_pose.orientation.x,
                              exp_pose.orientation.y,
                              exp_pose.orientation.z,
                              exp_pose.orientation.w])
        actual_ori = np.asarray([req_pose.orientation.x,
                                 req_pose.orientation.y,
                                 req_pose.orientation.z,
                                 req_pose.orientation.w])
        if np.sum(np.abs(exp_ori - actual_ori)) > quat_delta and \
                np.sum(np.abs(exp_ori + actual_ori)) > quat_delta:
            self.fail("quaternions do not match, expected " + str(exp_ori) + ", actual value is " + str(actual_ori))

    def _analyze_request_pose(self, exp_link, exp_pose, act_request, position_delta=None, quat_delta=DBL_EPSILON):
        # Check position constraint
        self.assertEquals(1, len(act_request.goal_constraints))
        self.assertEquals(1, len(act_request.goal_constraints[0].position_constraints))
        pose_con = act_request.goal_constraints[0].position_constraints[0]
        self.assertEquals(1, len(pose_con.constraint_region.primitive_poses))
        self.assertEquals(exp_link, pose_con.link_name)

        # Check orientation constraint, q and -q are the same rotation and thus both considered equal
        self.assertEquals(1, len(act_request.goal_constraints[0].orientation_constraints))
        ori_con = act_request.goal_constraints[0].orientation_constraints[0]
        self.assertEquals(exp_link, ori_con.link_name)

        request_pose = Pose(position=pose_con.constraint_region.primitive_poses[0].position,
                            orientation=ori_con.orientation)
        self._analyze_pose(exp_pose, request_pose, position_delta, quat_delta)



    def _analyze_request_relative_pose(self, exp_relative_position, exp_relative_euler, req):
        current_pose = self.robot.get_current_pose()
        current_euler = transformations.euler_from_quaternion([current_pose.orientation.x,
                                                               current_pose.orientation.y,
                                                               current_pose.orientation.z,
                                                               current_pose.orientation.w], axes=AXIS_SEQUENCE)

        exp_pose = Pose()
        exp_pose.position.x = current_pose.position.x + exp_relative_position.x
        exp_pose.position.y = current_pose.position.y + exp_relative_position.y
        exp_pose.position.z = current_pose.position.z + exp_relative_position.z

        exp_euler = [x + y for x, y in zip(current_euler, exp_relative_euler)]

        [exp_pose.orientation.x, exp_pose.orientation.y, exp_pose.orientation.z, exp_pose.orientation.w] = \
            transformations.quaternion_from_euler(exp_euler[0], exp_euler[1], exp_euler[2], AXIS_SEQUENCE)

        self._analyze_request_pose(TARGET_LINK_NAME, exp_pose, req)

    def _analyze_request_joint(self, exp_joint_names, exp_joint_values, act_request):
        self.assertEquals(1, len(act_request.goal_constraints))
        self.assertEquals(len(exp_joint_names), len(act_request.goal_constraints[0].joint_constraints))
        self.assertEquals(len(exp_joint_values), len(act_request.goal_constraints[0].joint_constraints))
        for exp_joint_name, exp_joint_value, joint_constraint in \
                zip(exp_joint_names, exp_joint_values, act_request.goal_constraints[0].joint_constraints):
            self.assertEquals(exp_joint_name, joint_constraint.joint_name)
            self.assertAlmostEqual(exp_joint_value, joint_constraint.position)

    def _analyze_request_circ_help_point(self, exp_help_name, exp_help_pose, req):
        self.assertEqual(exp_help_name,
                         req.path_constraints.name)
        req_position = req.path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position
        self.assertAlmostEqual(exp_help_pose.position.x,
                               req_position.x)
        self.assertAlmostEqual(exp_help_pose.position.y,
                               req_position.y)
        self.assertAlmostEqual(exp_help_pose.position.z,
                               req_position.z)

    def setUp(self):
        rospy.loginfo("Loading Robot...")
        self.robot = Robot(API_VERSION)
        rospy.loginfo("Loading Robot done")
        self.test_data = XmlTestdataLoader(_TEST_DATA_FILE_NAME)
        self.tf = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

    def tearDown(self):
        if hasattr(self, 'tf_listener'):
            self.tf_listener.clear()
        if hasattr(self, 'robot'):
            self.robot._release()
            self.robot = None
        if hasattr(self, 'test_data'):
            del self.test_data

    def test_ptp_cmd_convert_pose(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call ptp convert function with cartesian goal pose.

            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_goal_pose = self.test_data.get_pose("PTPPose", PLANNING_GROUP_NAME)

        ptp = Ptp(goal=exp_goal_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = ptp._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_goal_pose, req)

    def test_ptp_cmd_convert_pose_uninitialized_orientation(self):
        """ Check that conversion with uninitialized orientation.

            Test sequence:
                1. Move the robot to a start pose.
                2. Call ptp convert function with goal pose and the orientation is uninitialized.

            Test Results:
                1. Robot.SUCCESS is returned.
                2. Correct MotionPlanRequest is returned.
        """
        # 1
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        # 2
        exp_goal_pose = self.robot._robot_commander.get_group(PLANNING_GROUP_NAME).\
            get_current_pose(TARGET_LINK_NAME).pose

        input_goal_pose = Pose(position=exp_goal_pose.position, orientation=Quaternion())

        ptp = Ptp(goal=input_goal_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = ptp._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_goal_pose, req)

    def test_ptp_cmd_convert_joint(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call ptp convert function with joint goal.

            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_joint_values = self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)

        ptp = Ptp(goal=exp_joint_values, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = ptp._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, req)

        exp_joint_names = self.robot._robot_commander.get_group(req.group_name).get_active_joints()
        self._analyze_request_joint(exp_joint_names, exp_joint_values, req)

    def test_ptp_cmd_convert_negative(self):
        """ Check that conversion to MotionPlanRequest returns None.

            Test sequence:
                1. Call ptp convert function with no goal.
                2. Call ptp convert function with a joint goal, which has more joint values than needed.
                3. Call ptp convert function with goal of unknown type

            Test results:
                1. raises exception.
                2. raises exception.
                2. raises exception.
        """
        # 1
        ptp_1 = Ptp(vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NameError, ptp_1._cmd_to_request, self.robot)

        # 2
        exp_joint_values = self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)
        exp_joint_values.append(1)
        ptp_2 = Ptp(goal=exp_joint_values, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(IndexError, ptp_2._cmd_to_request, self.robot)

        #3
        ptp_3 = Ptp(goal=object(), vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NotImplementedError, ptp_3._cmd_to_request, self.robot)

    def test_ptp_relative_joint(self):
        """ Test the conversion of ptp command with relative joint works correctly

            Test sequence:
                1. Move the robot to start position via ptp.
                2. Call ptp convert function with a relative joint goal.

            Test results:
                1. Execution successful.
                2. Relative goal is correctly converted into an absolute goal.

        """
        joint_goal = self.test_data.get_joints("RelJoint", PLANNING_GROUP_NAME)

        # 1
        self.robot.move(Ptp(goal=joint_goal))

        # 2
        ptp_relative = Ptp(goal=joint_goal, vel_scale=1, acc_scale=1, relative=True)
        req = ptp_relative._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", 1, 1, req)

        # new expected joint position from current robot position
        exp_joint_values = [2*j for j in joint_goal]

        exp_joint_names = self.robot._robot_commander.get_group(req.group_name).get_active_joints()
        self._analyze_request_joint(exp_joint_names, exp_joint_values, req)

    def test_ptp_relative_pose_euler(self):
        """ Test the conversion of ptp command with relative pose works correctly

            Test sequence:
                1. Move the robot to start position via ptp.
                2. Call ptp convert function with a relative pose goal.

            Test results:
                1. Execution successful.
                2. Relative goal is correctly converted into an absolute goal.

        """
        # 1
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        # 2

        exp_relative_position = [0.1, 0.2, 0.3]
        for exp_relative_euler in EULER_SAMPLES_RELATIVE_ROTATION:
            # skip rotations > 180 degrees and samples with b==0
            if np.sum(np.abs(exp_relative_euler)) > pi or exp_relative_euler[1] == 0.0:
                continue

            # expected pose
            exp_relative_pose = Pose(position=Point(*exp_relative_position),
                                     orientation=from_euler(*exp_relative_euler))

            # conversion
            ptp = Ptp(goal=exp_relative_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE, relative=True)
            req = ptp._cmd_to_request(self.robot)
            self.assertIsNotNone(req)

            # check result
            self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
            self._analyze_request_relative_pose(exp_relative_pose.position, exp_relative_euler, req)

    def test_ptp_relative_pose_negative_euler(self):
        """ Test the conversion of ptp command with relative pose works correctly

            Test sequence:
                1. Move the robot to start position via ptp.
                2. Call ptp convert function with a relative pose goal.

            Test results:
                1. Execution successful.
                2. Relative goal is correctly converted into an absolute goal.

        """
        # 1
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))
        # 2
        # expected pose
        exp_relative_position = [0.1, 0.2, 0.3]
        exp_relative_euler = [-0.1, -0.2, -0.3]
        exp_relative_pose = Pose(position=Point(*exp_relative_position), orientation=from_euler(*exp_relative_euler))

        # conversion
        ptp = Ptp(goal=exp_relative_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE, relative=True)
        req = ptp._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        # check result
        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_relative_pose(exp_relative_pose.position, exp_relative_euler, req)

    def test_ptp_default_acc_scale(self):
        """ Test the default acceleration scaling factor for ptp commands.

            Test sequence:
                1. Create ptp command without given acceleration scaling factor.
                2. Call convert function.

            Test results:
                1. -
                2. Correct default acceleration scaling factor is set.
        """
        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        ptp = Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME), vel_scale=EXP_VEL_SCALE,
                  relative=True)
        req = ptp._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++
        self.assertEquals(EXP_VEL_SCALE*EXP_VEL_SCALE, req.max_acceleration_scaling_factor)

    def test_lin_cmd_convert_pose(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call lin convert function with cartesian goal pose.

            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_goal_pose = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)

        lin = Lin(goal=exp_goal_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = lin._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "LIN", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_goal_pose, req)

    def test_lin_cmd_convert_joint(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call lin convert function with joint goal.

            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_joint_values = self.test_data.get_joints("LINPose1", PLANNING_GROUP_NAME)

        lin = Lin(goal=exp_joint_values, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = lin._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "LIN", EXP_VEL_SCALE, EXP_ACC_SCALE, req)

        exp_joint_names = self.robot._robot_commander.get_group(req.group_name).get_active_joints()
        self._analyze_request_joint(exp_joint_names, exp_joint_values, req)

    def test_lin_cmd_convert_negative(self):
        """ Check that conversion to MotionPlanRequest returns None.

            Test sequence:
                1. Call lin convert function with no goal.
                2. Call lin convert function with a joint goal, which has less joint values than needed.

            Test Results:
                1. raises Exception.
                2. raises Exception.
        """
        # 1
        lin_1 = Lin(vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NameError, lin_1._cmd_to_request, self.robot)

        # 2
        exp_joint_values = self.test_data.get_joints("LINPose1", PLANNING_GROUP_NAME)
        lin_2 = Lin(goal=exp_joint_values[:-1], vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(IndexError, lin_2._cmd_to_request, self.robot)

    def test_lin_relative_joint(self):
        """ Test the conversion of lin command with relative joint works correctly

            Test sequence:
                1. Move the robot to start position via ptp.
                2. Call lin convert function with a relative joint goal.

            Test results:
                1. Execution successful.
                2. Relative goal is correctly converted into an absolute goal.

        """
        joint_goal = self.test_data.get_joints("RelJoint", PLANNING_GROUP_NAME)

        # 1
        self.robot.move(Ptp(goal=joint_goal))

        # 2
        lin_relative = Lin(goal=joint_goal, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE, relative=True)
        req = lin_relative._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        self._analyze_request_general(PLANNING_GROUP_NAME, "LIN", EXP_VEL_SCALE, EXP_ACC_SCALE, req)

        # new expected joint position from current robot position
        exp_joint_values = [2*j for j in joint_goal]

        exp_joint_names = self.robot._robot_commander.get_group(req.group_name).get_active_joints()
        self._analyze_request_joint(exp_joint_names, exp_joint_values, req)

    def test_lin_relative_pose_euler(self):
        """ Test the conversion of lin command with relative pose works correctly

            Test sequence:
                1. Move the robot to start position via ptp.
                2. Call ptp convert function with a relative pose goal.

            Test results:
                1. Execution successful.
                2. Relative goal is correctly converted into an absolute goal.

        """
        # 1
        self.robot.move(Ptp(goal=self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)))

        # 2

        exp_relative_position = [0.1, 0.2, 0.3]
        for exp_relative_euler in EULER_SAMPLES_RELATIVE_ROTATION:
            # skip rotations > 180 degrees and samples with b==0
            if np.sum(np.abs(exp_relative_euler)) > pi or exp_relative_euler[1] == 0.0:
                continue

            # expected pose
            exp_relative_pose = Pose(position=Point(*exp_relative_position),
                                     orientation=from_euler(*exp_relative_euler))

            # conversion
            lin = Lin(goal=exp_relative_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE, relative=True)
            req = lin._cmd_to_request(self.robot)
            self.assertIsNotNone(req)

            # check result
            self._analyze_request_general(PLANNING_GROUP_NAME, "LIN", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
            self._analyze_request_relative_pose(exp_relative_pose.position, exp_relative_euler, req)

    def test_lin_default_acc_scale(self):
        """ Test the default acceleration scaling factor for lin commands.

            Test sequence:
                1. Create lin command without given acceleration scaling factor.
                2. Call convert function.

            Test results:
                1. -
                2. Correct default acceleration scaling factor is set.
        """
        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        lin = Lin(goal=self.test_data.get_joints("LINPose1", PLANNING_GROUP_NAME), vel_scale=EXP_VEL_SCALE,
                  relative=True)
        req = lin._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++
        self.assertEquals(EXP_VEL_SCALE, req.max_acceleration_scaling_factor)

    def test_circ_cmd_convert_center(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call circ convert function with center point.


            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_goal_pose = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_help_pose = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        circ = Circ(goal=exp_goal_pose, center=exp_help_pose.position, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = circ._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "CIRC", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_goal_pose, req)
        self._analyze_request_circ_help_point("center", exp_help_pose, req)

    def test_circ_cmd_convert_interim(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call circ convert function with interim point.

            Test Results:
                1. Correct MotionPlanRequest is returned.
            """
        exp_goal_pose = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_help_pose = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        circ = Circ(goal=exp_goal_pose, interim=exp_help_pose.position,
                    vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        req = circ._cmd_to_request(self.robot)
        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "CIRC", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_goal_pose, req)
        self._analyze_request_circ_help_point("interim", exp_help_pose, req)

    def test_circ_cmd_convert_joint(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call circ convert function with joint goal and center point.


            Test Results:
                1. Correct MotionPlanRequest is returned.
        """
        exp_joint_values = self.test_data.get_joints("LINPose1", PLANNING_GROUP_NAME)
        exp_help_pose = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        circ = Circ(goal=exp_joint_values, center=exp_help_pose.position, vel_scale=EXP_VEL_SCALE,
                    acc_scale=EXP_ACC_SCALE)
        req = circ._cmd_to_request(self.robot)

        self.assertIsNotNone(req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "CIRC", EXP_VEL_SCALE, EXP_ACC_SCALE, req)
        exp_joint_names = self.robot._robot_commander.get_group(req.group_name).get_active_joints()
        self._analyze_request_joint(exp_joint_names, exp_joint_values, req)
        self._analyze_request_circ_help_point("center", exp_help_pose, req)

    def test_circ_cmd_convert_negative(self):
        """ Check that conversion to MotionPlanRequest works correctly.

            Test sequence:
                1. Call circ convert function with no pose goal.
                2. Call circ convert function with no help point.
                3. Call circ convert function with both help points.

            Test Results:
                1. raises Exception.
                2. raises Exception.
                3. raises Exception.
        """
        exp_goal_pose = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_help_pose = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        # 1
        circ_1 = Circ(interim=exp_help_pose.position, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NameError, circ_1._cmd_to_request, self.robot)

        # 2
        circ_2 = Circ(goal=exp_goal_pose, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NameError, circ_2._cmd_to_request, self.robot)

        # 3
        circ_3 = Circ(goal=exp_goal_pose, center=exp_help_pose.position, interim=exp_help_pose.position,
                      vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        self.assertRaises(NameError, circ_3._cmd_to_request, self.robot)

    def test_circ_default_acc_scale(self):
        """ Test the default acceleration scaling factor for circ commands.

            Test sequence:
                1. Create circ command without given acceleration scaling factor.
                2. Call convert function.

            Test results:
                1. -
                2. Correct default acceleration scaling factor is set.
        """
        # +++++++++++++++++++++++
        rospy.loginfo("Step 1")
        # +++++++++++++++++++++++
        exp_goal_pose = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_help_pose = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        circ = Circ(goal=exp_goal_pose, center=exp_help_pose.position, vel_scale=EXP_VEL_SCALE)

        req = circ._cmd_to_request(self.robot)
        self.assertIsNotNone(req)

        # +++++++++++++++++++++++
        rospy.loginfo("Step 2")
        # +++++++++++++++++++++++
        self.assertEquals(EXP_VEL_SCALE, req.max_acceleration_scaling_factor)

    def test_get_sequence_request(self):
        """ Test the _get_sequence_request function of Sequence command works correctly.

            Test sequence:
                1. Append valid ptp, lin and circ commands with blend radius to Sequence command
                2. Call _get_sequence_request function of the Sequence command

            Test results:
                1. -
                2. Correct MotionSequenceRequest is returned
        """
        # 1
        exp_ptp_goal = self.test_data.get_joints("PTPJointValid", PLANNING_GROUP_NAME)
        exp_lin_goal = self.test_data.get_pose("LINPose2", PLANNING_GROUP_NAME)
        exp_circ_goal = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_circ_center = self.test_data.get_pose("CIRCCenterPose", PLANNING_GROUP_NAME)

        exp_blend_radius_1 = 0.1
        exp_blend_radius_2 = 0.0
        exp_blend_radius_3 = 0.3

        exp_joint_names = self.robot._robot_commander.get_group(PLANNING_GROUP_NAME).get_active_joints()

        ptp = Ptp(goal=exp_ptp_goal, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        lin = Lin(goal=exp_lin_goal, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        circ = Circ(goal=exp_circ_goal, center=exp_circ_center.position, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)

        seq = Sequence()
        seq.append(ptp, exp_blend_radius_1)
        seq.append(lin, exp_blend_radius_2)
        seq.append(circ, exp_blend_radius_3)

        # 2
        seq_action_goal = seq._get_sequence_request(self.robot)

        self.assertIsNotNone(seq_action_goal)
        self.assertEqual(3, len(seq_action_goal.request.items))

        # check first command (ptp)
        self.assertEqual(exp_blend_radius_1, seq_action_goal.request.items[0].blend_radius)
        ptp_req = seq_action_goal.request.items[0].req

        self.assertIsNotNone(ptp_req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "PTP", EXP_VEL_SCALE, EXP_ACC_SCALE, ptp_req)
        self._analyze_request_joint(exp_joint_names, exp_ptp_goal, ptp_req)

        # check second command (lin)
        self.assertEqual(exp_blend_radius_2, seq_action_goal.request.items[1].blend_radius)
        lin_req = seq_action_goal.request.items[1].req

        self.assertIsNotNone(lin_req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "LIN", EXP_VEL_SCALE, EXP_ACC_SCALE, lin_req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_lin_goal, lin_req)

        # check third command (circ)
        self.assertEqual(exp_blend_radius_3, seq_action_goal.request.items[2].blend_radius)
        circ_req = seq_action_goal.request.items[2].req

        self.assertIsNotNone(circ_req)
        self._analyze_request_general(PLANNING_GROUP_NAME, "CIRC", EXP_VEL_SCALE, EXP_ACC_SCALE, circ_req)
        self._analyze_request_pose(TARGET_LINK_NAME, exp_circ_goal, circ_req)
        self._analyze_request_circ_help_point("center", exp_circ_center, circ_req)

    def test_get_sequence_request_negative(self):
        """ Test the _get_sequence_request function of Sequence command works correctly.

            Test sequence:
                1. Append one valid and one invalid lin command with blend radius to Sequence command
                2. Call _get_sequence_request function of the Sequence command

            Test results:
                1. -
                2. raises Exception
        """
        # 1
        exp_goal_pose_1 = self.test_data.get_pose("LINPose1", PLANNING_GROUP_NAME)
        exp_blend_radius_1 = 0.1
        exp_blend_radius_2 = 0

        lin_1 = Lin(goal=exp_goal_pose_1, vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)
        lin_2 = Lin(vel_scale=EXP_VEL_SCALE, acc_scale=EXP_ACC_SCALE)

        seq = Sequence()
        seq.append(lin_1, exp_blend_radius_1)
        seq.append(lin_2, exp_blend_radius_2)

        # 2
        self.assertRaises(NameError, seq._get_sequence_request, self.robot)

    def test_relative_misusage(self):
        """ Test if using reference frame in strange ways results in the correct response.

            Test sequence:
                1. Get the test data and publish the reference frames via TF.
                2. Create Ptp with special use cases.
                3. Call the _cmd_to_request function to convert the goal.
                4. Evaluate the results.

        """
        rospy.loginfo("test_relative_misusage")

        # test data in base frame
        goal_pose_bf = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)
        self.robot.move(Ptp(goal=goal_pose_bf))

        # no rotation assigned: should interpret the rotation as Quaternion(0, 0, 0, 1)
        no_rot = Ptp(goal=Pose(position=Point(0, 0, - 0.1)), relative=True)
        no_rot_req = no_rot._cmd_to_request(self.robot)

        goal_pose_bf.position.z -= 0.1
        self.assertIsNotNone(no_rot_req)
        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, no_rot_req)

    def test_custom_reference_frame_lin_ptp_circ(self):
        """ Test the command conversion works correctly if a goal is given in Cartesian space
        with a reference frame.

            Test sequence:
                1. Get the test data and publish the reference frames via TF.
                3. Use the tf listener to read the transform between the tfs
                2. Create a ptp command with the published reference frame as reference and the transform as goal pose.
                3. Call the _cmd_to_request function to convert the goal.
                4. Evaluate the result.

            Test results:
                The goal constraint in motion plan request should equal to the goal pose in base frame.

        """
        rospy.loginfo("test_custom_reference_frame")

        self.robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))

        # read and transform test data
        ref_frame = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)
        goal_pose_bf = self.test_data.get_pose("Blend_1_Start", PLANNING_GROUP_NAME)

        goal_pose_bf_tf = pm.toTf(pm.fromMsg(goal_pose_bf))
        ref_frame_tf = pm.toTf(pm.fromMsg(ref_frame))

        rospy.sleep(rospy.Duration.from_sec(0.5))

        # init
        time_tf = rospy.Time.now()
        goal_pose_in_rf = [[0, 0, 0], [0, 0, 0, 1]]
        base_frame_name = self.robot._robot_commander.get_planning_frame()

        try:
            self.tf.sendTransform(goal_pose_bf_tf[0], goal_pose_bf_tf[1], time_tf,
                                  "goal_pose_bf", base_frame_name)
            self.tf.sendTransform(ref_frame_tf[0], ref_frame_tf[1], time_tf,
                                  "ref_move_frame", base_frame_name)

            # look up the relative pose in reference frame of the goal
            self.tf_listener.waitForTransform("goal_pose_bf", "ref_move_frame", rospy.Time(0), rospy.Duration(2, 0))

            rospy.sleep(rospy.Duration.from_sec(0.1))

            goal_pose_in_rf = self.tf_listener.lookupTransform("ref_move_frame", "goal_pose_bf", rospy.Time(0))
        except:
            rospy.logerr("Failed to setup transforms for test!")

        goal_pose_rf_msg = pm.toMsg(pm.fromTf(goal_pose_in_rf))

        # convert the goal in reference frame to planning request(goal in base frame)
        ptp = Ptp(goal=goal_pose_rf_msg, reference_frame="ref_move_frame")
        request_ptp = ptp._cmd_to_request(self.robot)

        circ = Circ(goal=goal_pose_rf_msg, interim=goal_pose_rf_msg.position, reference_frame="ref_move_frame")
        request_circ = circ._cmd_to_request(self.robot)

        lin = Lin(goal=goal_pose_rf_msg, reference_frame="ref_move_frame")
        request_lin = lin._cmd_to_request(self.robot)

        self.assertIsNotNone(request_ptp)
        self.assertIsNotNone(request_circ)
        self.assertIsNotNone(request_lin)

        # evaluate the result
        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, request_ptp, TF_POSITION_COMPARE_DELTA,
                                   TF_QUATERNION_COMPARE_DELTA)

        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, request_lin, TF_POSITION_COMPARE_DELTA,
                                   TF_QUATERNION_COMPARE_DELTA)

        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, request_circ, TF_POSITION_COMPARE_DELTA,
                                   TF_QUATERNION_COMPARE_DELTA)
        self._analyze_request_circ_help_point("interim", goal_pose_bf, request_circ)

    def test_custom_reference_frame_misusage(self):
        """ Test if using reference frame in strange ways results in the correct response.

            Test sequence:
                1. Get the test data and publish the reference frames via TF.
                2. Create Ptp with special use cases.
                3. Call the _cmd_to_request function to convert the goal.
                4. Evaluate the results.

        """
        rospy.loginfo("test_custom_reference_frame_misusage")

        # test data in base frame
        goal_pose_bf = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)

        # frame equals base frame: should return identity
        same = Ptp(goal=goal_pose_bf, reference_frame=self.robot._robot_commander.get_planning_frame())
        request_same = same._cmd_to_request(self.robot)

        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, request_same)

        # test with non existing frame: should return none
        ptp_no_tf = Ptp(goal=goal_pose_bf, reference_frame="this_does_not_exist")

        self.assertRaises(RobotCurrentStateError, ptp_no_tf._cmd_to_request, self.robot)

        # no rotation assigned: should interpret the rotation as Quaternion(0, 0, 0, 1)
        no_rot = Ptp(goal=Pose(position=Point(0, 0, 1)),
                     reference_frame=self.robot._robot_commander.get_planning_frame())
        no_rot_req = no_rot._cmd_to_request(self.robot)

        self._analyze_request_pose(TARGET_LINK_NAME, Pose(position=Point(0, 0, 1), orientation=Quaternion(0, 0, 0, 1)),
                                   no_rot_req)

    def test_custom_reference_relative_move(self):
        """ Test if relative moves work with custom reference frame as expected

            Test sequence:
                1. Get the test data and publish reference frame via TF.
                2. Create a relative Ptp with and without custom reference.
                3. convert the goals.
                4. Evaluate the results.
        """
        rospy.loginfo("test_custom_reference_frame_relative")

        self.robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))

        # get and transform test data
        ref_frame = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)
        goal_pose_bf = self.test_data.get_pose("Blend_1_Start", PLANNING_GROUP_NAME)
        goal_pose_bf_tf = pm.toTf(pm.fromMsg(goal_pose_bf))
        ref_frame_tf = pm.toTf(pm.fromMsg(ref_frame))

        rospy.sleep(rospy.Duration.from_sec(0.5))

        # init
        base_frame_name = self.robot._robot_commander.get_planning_frame()
        time_tf = rospy.Time.now()
        goal_pose_in_rf = [[0, 0, 0], [0, 0, 0, 1]]

        try:
            self.tf.sendTransform(goal_pose_bf_tf[0], goal_pose_bf_tf[1], time_tf,
                                  "rel_goal_pose_bf", base_frame_name)
            self.tf.sendTransform(ref_frame_tf[0], ref_frame_tf[1], time_tf,
                                  "ref_rel_frame", base_frame_name)

            self.tf_listener.waitForTransform("rel_goal_pose_bf", "ref_rel_frame", time_tf, rospy.Duration(2, 0))

            rospy.sleep(rospy.Duration.from_sec(0.1))

            goal_pose_in_rf = self.tf_listener.lookupTransform("ref_rel_frame", "rel_goal_pose_bf", time_tf)
        except:
            rospy.logerr("Failed to setup transforms for test!")

        goal_pose_rf_msg = pm.toMsg(pm.fromTf(goal_pose_in_rf))

        # move to initial position and use relative move to reach goal
        self.robot.move(Ptp(goal=ref_frame))

        self.tf.sendTransform(ref_frame_tf[0], ref_frame_tf[1], rospy.Time.now(), "ref_rel_frame", base_frame_name)
        rospy.sleep(rospy.Duration.from_sec(0.1))
        self.tf_listener.waitForTransform(base_frame_name, "ref_rel_frame", rospy.Time(0), rospy.Duration(1, 0))

        ptp = Ptp(goal=goal_pose_rf_msg, reference_frame="ref_rel_frame", relative=True)
        req = ptp._cmd_to_request(self.robot)

        self.assertIsNotNone(req)
        self._analyze_request_pose(TARGET_LINK_NAME, goal_pose_bf, req)

    def test_current_pose(self):
        """ Test the current pose method

            1. create and publish tf
            2. get current pose with base and ref
            3. move roboter to ref
            4. get current pose with ref and base
            5. analyse positions
        """
        rospy.loginfo("test_current_pose")

        self.robot.move(Ptp(goal=[0, 0, 0, 0, 0, 0]))

        # get and transform test data
        ref_frame = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)
        ref_frame_tf = pm.toTf(pm.fromMsg(ref_frame))

        rospy.sleep(rospy.Duration.from_sec(0.5))

        # init
        tcp_ref = [[0, 0, 0], [0, 0, 0, 1]]
        tcp_base = [[0, 0, 0], [0, 0, 0, 1]]
        time_tf = rospy.Time.now()
        base_frame = self.robot._robot_commander.get_planning_frame()

        try:
            self.tf.sendTransform(ref_frame_tf[0], ref_frame_tf[1],
                                  time_tf, "ref_frame", base_frame)

            self.tf_listener.waitForTransform(base_frame, "ref_frame", time_tf, rospy.Duration(2, 0))
            rospy.sleep(rospy.Duration.from_sec(0.1))
            tcp_ref = self.tf_listener.lookupTransform("ref_frame", "prbt_tcp", time_tf)
            tcp_base = self.tf_listener.lookupTransform(base_frame, "prbt_tcp", time_tf)
        except Exception as e:
            print e
            rospy.logerr("Failed to setup transforms for test!")

        tcp_ref_msg = pm.toMsg(pm.fromTf(tcp_ref))
        tcp_base_msg = pm.toMsg(pm.fromTf(tcp_base))

        # read current pose, move robot and do it again
        start_bf = self.robot.get_current_pose()
        start_rf = self.robot.get_current_pose(base="ref_frame")

        self.robot.move(Ptp(goal=ref_frame))

        # resending tf (otherwise transform pose could read old transform to keep time close together.
        time_tf = rospy.Time.now()
        self.tf.sendTransform(ref_frame_tf[0], ref_frame_tf[1], time_tf, "ref_frame", base_frame)
        self.tf_listener.waitForTransform(base_frame, "ref_frame", time_tf, rospy.Duration(1, 0))

        ref_frame_bf = self.robot.get_current_pose()
        ref_frame_rf = self.robot.get_current_pose(base="ref_frame")

        self._analyze_pose(tcp_base_msg, start_bf)
        self._analyze_pose(tcp_ref_msg, start_rf)
        self._analyze_pose(ref_frame, ref_frame_bf)
        self._analyze_pose(Pose(orientation=Quaternion(0, 0, 0, 1)), ref_frame_rf)

    def test_to_string(self):
        """ Test if to string functions cause errors

            Test sequence:
                1. create Ptp, Lin, Circ command
                2. convert the command => should not cause errors
        """
        pose = self.test_data.get_pose("Blend_1_Mid", PLANNING_GROUP_NAME)
        ref = self.robot._robot_commander.get_planning_frame()

        str(Ptp(goal=pose, relative=True,
            reference_frame=ref, vel_scale=0.2, acc_scale=0.2))
        str(Ptp(goal=[0, 0, 0, 0, 0, 0], relative=True,
            reference_frame=ref, vel_scale=0.2, acc_scale=0.2))

        str(Circ(goal=pose, interim=pose.position,
             reference_frame=ref, vel_scale=0.2, acc_scale=0.2))
        str(Circ(goal=pose, center=pose.position,
             reference_frame=ref, vel_scale=0.2, acc_scale=0.2))

        str(Lin(goal=pose, relative=True,
            reference_frame=ref, vel_scale=0.2, acc_scale=0.2))
        str(Lin(goal=[0, 0, 0, 0, 0, 0], relative=True,
            reference_frame=ref, vel_scale=0.2, acc_scale=0.2))

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_api_cmd_conversion')
    rostest.rosrun('pilz_robot_programming', 'test_api_cmd_conversion', TestAPICmdConversion)
