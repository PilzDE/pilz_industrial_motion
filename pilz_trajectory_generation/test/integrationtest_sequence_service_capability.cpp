/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <ros/ros.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include "pilz_msgs/GetMotionSequence.h"
#include "pilz_msgs/MotionSequenceRequest.h"
#include "pilz_trajectory_generation/capability_names.h"
#include "test_utils.h"

#include "motion_plan_request_builder.h"
#include "motion_sequence_request_builder.h"

const std::string BLEND_DATA_PREFIX("test_data/");

// Parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string BLEND_DATASET_NUM("blend_dataset_num");

class IntegrationTestSequenceService : public ::testing::Test
{
protected:
  virtual void SetUp();
  virtual void TearDown() {}

private:
  void generateDataSet();

protected:
  ros::NodeHandle ph_ {"~"};
  ros::ServiceClient client_;
  robot_model::RobotModelPtr robot_model_;
  std::vector<testutils::blend_test_data> test_data_;
  std::string planning_group_, target_link_;

  pilz_msgs::MotionSequenceRequest blend_command_list_2_, blend_command_list_3_;
};

void IntegrationTestSequenceService::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));

  // create robot model
  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // get test data set
  int blend_dataset_num;
  ASSERT_TRUE(ph_.getParam(BLEND_DATASET_NUM, blend_dataset_num));
  ASSERT_TRUE(testutils::getBlendTestData(ph_, blend_dataset_num, BLEND_DATA_PREFIX, test_data_));

  generateDataSet();

  ASSERT_TRUE(ros::service::waitForService(pilz_trajectory_generation::SEQUENCE_SERVICE_NAME, ros::Duration(10))) << "Service not available.";
  ros::NodeHandle nh; // connect to service in global namespace, not in ph_
  client_ = nh.serviceClient<pilz_msgs::GetMotionSequence>(pilz_trajectory_generation::SEQUENCE_SERVICE_NAME);
}

void IntegrationTestSequenceService::generateDataSet()
{
  geometry_msgs::PoseStamped p1, p2, p3;

  p1.header.frame_id = robot_model_->getModelFrame();
  p1.pose.position.x = 0.25;
  p1.pose.position.y = 0.3;
  p1.pose.position.z = 0.65;
  p1.pose.orientation.x = 0.;
  p1.pose.orientation.y = 0.;
  p1.pose.orientation.z = 0.;
  p1.pose.orientation.w = 1.;

  p2 = p1;
  p2.pose.position.x -= 0.15;

  // TODO: move p3 to testUtils
  p3 = p1;
  p3.pose.position.y -= 0.15;

  MotionPlanRequestBuilder builder("manipulator");

  // goal 1 (with start state)
  sensor_msgs::JointState start_state_joint = testutils::generateJointState({0., 0.007881892504574495, -1.8157263253868452,
    0., 1.8236082178909834, 0.});
  builder.setJointStartState(start_state_joint);
  builder.setGoal(target_link_, p1);

  moveit_msgs::MotionPlanRequest  req1, req2, req3;

  req1 = builder.createLin();

  // Clear the start state for the next goals
  builder.clearJointStartState();

  // goal 2
  builder.setGoal(target_link_, p2);
  req2 = builder.createLin();

  // goal3
  builder.setGoal(target_link_, p3);
  req3 = builder.createLin();

  MotionSequenceRequestBuilder blend_list_builder;
  blend_command_list_2_ = blend_list_builder.build({std::make_pair(req1, 0.08),
                                                    std::make_pair(req2, 0)});

  blend_command_list_3_ = blend_list_builder.build({std::make_pair(req1, 0.08),
                                                    std::make_pair(req2, 0.05),
                                                    std::make_pair(req3, 0)});
}

/**
 * @brief  Tests planning of two linear trajectories using the blend service.
 *
 * Test Sequence:
 *    1. Call blend service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Error code of the blend result is success.
 */
TEST_F(IntegrationTestSequenceService, blendLINLIN)
{
  for(const auto& test_data : test_data_)
  {
    pilz_msgs::MotionSequenceRequest req_list;
    testutils::generateRequestMsgFromBlendTestData(robot_model_, test_data,
                                                   "LIN", planning_group_,
                                                   target_link_, req_list);

    pilz_msgs::GetMotionSequence srv;
    srv.request.commands = req_list;

    // Call the service client
    ASSERT_TRUE(client_.call(srv));

    // Obtain the response
    const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

    // Make sure the planning succeeded
    EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning of blend trajectory failed!";
    EXPECT_GT(response.trajectory.joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
  }
}

/**
 * @brief Sends a blend request with negative blend_radius.
 * Checks if response is obtained and has the correct error code set.
 *
 * Test Sequence:
 *    1. Generate request, first goal has negative blend_radius +  Call blend service.
 *    2. Evaluate the result
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. blend fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, blendRadiusNegative)
{
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_2_;
  req_list.items[0].blend_radius = -0.3;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  // Call the service client
  ASSERT_TRUE(client_.call(srv));

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::FAILURE, response.error_code.val) << "Planning should have failed but did not.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Sends an empty blend request. Checks if response is obtained.
 *
 *  Test Sequence:
 *    1. Generate empty request and call blend service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. blend is successful, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, emptyList)
{
  pilz_msgs::MotionSequenceRequest empty_list;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = empty_list;

  // Call the service client
  ASSERT_TRUE(client_.call(srv));

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, response.error_code.val) << "Planning did failed.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Sends a blend request with invalid second start state.
 * Checks if response is obtained and has the correct error code set.
 *
 *  Test Sequence:
 *    1. Generate request, second goal has an invalid start state set +  Call blend service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. blend fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, startStateNotFirstGoal)
{
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_2_;
  req_list.items[1].req.start_state.joint_state = testutils::generateJointState({-1., 2., -3., 4., -5., 0.});

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  // Call the service client
  ASSERT_TRUE(client_.call(srv));

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS, response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Sends a blend request with invalid first goal.
 * Checks if response is obtained and has the correct error code set.
 *
 *  Test Sequence:
 *    1. Generate request with first goal out of workspace +  Call blend service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. blend fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, firstGoalNotReachable)
{
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_2_;
  req_list.items[0].req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = 27;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  // Call the service client
  ASSERT_TRUE(client_.call(srv));

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION, response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Tests the blend of more than two commands.
 *
 * Test Sequence:
 *    1. Call blend service with serveral requests.
 *    2. Evaluate the result
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Blend succeeds, result trajectory is not empty.
 */
TEST_F(IntegrationTestSequenceService, largeRequest)
{
  // construct request
  constexpr int N = 10;
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_3_;
  req_list.items.back().blend_radius = 0.01;
  for(int i = 0; i < N; ++i)
  {
    req_list.items.push_back(req_list.items[0]);
    req_list.items.back().req.start_state.joint_state.position.clear();
    req_list.items.back().req.start_state.joint_state.velocity.clear();
    req_list.items.back().req.start_state.joint_state.name.clear();
    req_list.items.push_back(req_list.items[1]);
    req_list.items.push_back(req_list.items[2]);
  }
  req_list.items.back().blend_radius = 0.0;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  // Call the service client
  ASSERT_TRUE(client_.call(srv));

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.plan_response;

  // Check result
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Incorrect error code.";
  EXPECT_GT(response.trajectory.joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_sequence_service_capability");
  ros::NodeHandle nh();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
