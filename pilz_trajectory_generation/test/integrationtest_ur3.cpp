/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/JointConstraint.h>

#include <pilz_industrial_motion_testutils/command_types_typedef.h>
#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>

#include "test_utils.h"

const double EPSILON = 1.0e-6;
const std::string PLAN_SERVICE_NAME = "/plan_kinematic_path";

// Parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");
const std::string ORIENTATION_NORM_TOLERANCE("orientation_norm_tolerance");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz_industrial_motion_testutils;

/**
 * PLEASE NOTE:
 * More detailed lin tests are done via unit tests. With the help of the
 * integration tests, it is only checked that a linear command actually
 * performs a linear command.
 */
class IntegrationTestCommandPlanning : public ::testing::Test {
protected:
  void SetUp() override;

protected:
  ros::NodeHandle ph_{"~"};

  robot_model::RobotModelPtr robot_model_;

  double pose_norm_tolerance_, orientation_norm_tolerance_;
  std::string planning_group_, target_link_, test_data_file_name_;

  std::unique_ptr<pilz_industrial_motion_testutils::TestdataLoader> test_data_;

  unsigned int num_joints_{0};
};

void IntegrationTestCommandPlanning::SetUp() {
  // create robot model
  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  // get the parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(
      ph_.getParam(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(
      ph_.getParam(ORIENTATION_NORM_TOLERANCE, orientation_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  // load the test data provider
  test_data_.reset(new pilz_industrial_motion_testutils::XmlTestdataLoader{
      test_data_file_name_, robot_model_});
  ASSERT_NE(nullptr, test_data_) << "Failed to load test data by provider.";

  num_joints_ = robot_model_->getJointModelGroup(planning_group_)
                    ->getActiveJointModelNames()
                    .size();
}

/**
 * @brief Tests if ptp motions with start & goal state given as
 * joint configuration are executed correctly.
 *
 * Test Sequence:
 *    1. Generate request with joint goal and start state call planning service.
 *
 * Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 */
TEST_F(IntegrationTestCommandPlanning, PtpJoint) {
  ros::NodeHandle node_handle("~");
  auto ptp{test_data_->getPtpJoint("Ptp1")};

  moveit_msgs::GetMotionPlan srv;
  moveit_msgs::MotionPlanRequest req = ptp.toRequest();
  srv.request.motion_plan_request = req;

  ASSERT_TRUE(ros::service::waitForService(
      PLAN_SERVICE_NAME, ros::Duration(testutils::DEFAULT_SERVICE_TIMEOUT)));
  ros::ServiceClient client =
      node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  ASSERT_TRUE(client.call(srv));
  const moveit_msgs::MotionPlanResponse &response{
      srv.response.motion_plan_response};

  // Check the result
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val)
      << "Planning failed!";
  trajectory_msgs::JointTrajectory trajectory =
      response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_)
      << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0u)
      << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for (trajectory_msgs::JointTrajectoryPoint point : trajectory.points) {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    EXPECT_NEAR(trajectory.points.back().positions.at(i),
                req.goal_constraints.back().joint_constraints.at(i).position,
                10e-10);
    EXPECT_NEAR(trajectory.points.back().velocities.at(i), 0, 10e-10);
    // EXPECT_NEAR(trajectory.points.back().accelerations.at(i), 0, 10e-10); //
    // TODO what is expected
  }
}

/**
 * @brief Tests if ptp motions with start state given as joint configuration
 * and goal state given as cartesian configuration are executed correctly.
 *
 * Test Sequence:
 *    1. Generate request with pose goal and start state call planning service.
 *
 * Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 */
TEST_F(IntegrationTestCommandPlanning, PtpJointCart) {
  ros::NodeHandle node_handle("~");

  PtpJointCart ptp{test_data_->getPtpJointCart("Ptp1")};
  ptp.getGoalConfiguration().setPoseTolerance(0.01);
  ptp.getGoalConfiguration().setAngleTolerance(0.01);

  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = ptp.toRequest();

  ASSERT_TRUE(ros::service::waitForService(
      PLAN_SERVICE_NAME, ros::Duration(testutils::DEFAULT_SERVICE_TIMEOUT)));
  ros::ServiceClient client =
      node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  ASSERT_TRUE(client.call(srv));
  const moveit_msgs::MotionPlanResponse &response{
      srv.response.motion_plan_response};

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val)
      << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory =
      response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_)
      << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0u)
      << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for (trajectory_msgs::JointTrajectoryPoint point : trajectory.points) {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  // TODO check that at right position
  robot_state::RobotState rstate(robot_model_);
  rstate.setJointGroupPositions(
      planning_group_,
      response.trajectory.joint_trajectory.points.back().positions);
  rstate.update();
  Eigen::Isometry3d tf = rstate.getFrameTransform(target_link_);

  const geometry_msgs::Pose &expected_pose{
      ptp.getGoalConfiguration().getPose()};
  EXPECT_NEAR(tf(0, 3), expected_pose.position.x, EPSILON);
  EXPECT_NEAR(tf(1, 3), expected_pose.position.y, EPSILON);
  EXPECT_NEAR(tf(2, 3), expected_pose.position.z, EPSILON);

  Eigen::Isometry3d exp_iso3d_pose;
  tf::poseMsgToEigen(expected_pose, exp_iso3d_pose);

  EXPECT_TRUE(
      Eigen::Quaterniond(tf.rotation())
          .isApprox(Eigen::Quaterniond(exp_iso3d_pose.rotation()), EPSILON));
}

TEST_F(IntegrationTestCommandPlanning, PtpCart) {
  ros::NodeHandle node_handle("~");

  PtpCart ptp{test_data_->getPtpCart("Ptp2")};
  ptp.getGoalConfiguration().setPoseTolerance(0.01);
  ptp.getGoalConfiguration().setAngleTolerance(0.01);

  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = ptp.toRequest();

  ASSERT_TRUE(ros::service::waitForService(
      PLAN_SERVICE_NAME, ros::Duration(testutils::DEFAULT_SERVICE_TIMEOUT)));
  ros::ServiceClient client =
      node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  ASSERT_TRUE(client.call(srv));
  const moveit_msgs::MotionPlanResponse &response{
      srv.response.motion_plan_response};

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val)
      << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory =
      response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_)
      << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0u)
      << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for (trajectory_msgs::JointTrajectoryPoint point : trajectory.points) {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  // TODO check that at right position
  robot_state::RobotState rstate(robot_model_);
  rstate.setJointGroupPositions(
      planning_group_,
      response.trajectory.joint_trajectory.points.back().positions);
  rstate.update();
  Eigen::Isometry3d tf = rstate.getFrameTransform(target_link_);

  const geometry_msgs::Pose &expected_pose{
      ptp.getGoalConfiguration().getPose()};
  EXPECT_NEAR(tf(0, 3), expected_pose.position.x, EPSILON);
  EXPECT_NEAR(tf(1, 3), expected_pose.position.y, EPSILON);
  EXPECT_NEAR(tf(2, 3), expected_pose.position.z, EPSILON);

  Eigen::Isometry3d exp_iso3d_pose;
  tf::poseMsgToEigen(expected_pose, exp_iso3d_pose);

  EXPECT_TRUE(
      Eigen::Quaterniond(tf.rotation())
          .isApprox(Eigen::Quaterniond(exp_iso3d_pose.rotation()), EPSILON));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "integrationtest_ur3");
  ros::NodeHandle nh; // For output via ROS_ERROR etc during test

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
