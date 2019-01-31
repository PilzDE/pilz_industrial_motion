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
#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanResponse.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/sequence.h>

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
const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using namespace pilz_industrial_motion_testutils;

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

  std::string test_data_file_name_;
  TestdataLoaderUPtr data_loader_;
};

void IntegrationTestSequenceService::SetUp()
{
  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  // create robot model
  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // load the test data provider
  data_loader_.reset(new XmlTestdataLoader(test_data_file_name_, robot_model_));
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

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
 * @brief Test behavior of service when empty sequence is sent.
 *
 *  Test Sequence:
 *    1. Generate empty request and call sequence service.
 *    2. Evaluate the result.
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command is successful, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSendingOfEmptySequence)
{
  pilz_msgs::MotionSequenceRequest empty_list;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = empty_list;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Tests that invalid (differing) group names are detected.
 *
 * Test Sequence:
 *    1. Generate request, first request has invalid group_name +  Call sequence service.
 *    2. Invalidate first request (change group_name) and send goal for planning and execution.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestDifferingGroupNames)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  req.items.at(0).req.group_name = "WrongGroupName";

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME, response.error_code.val) << "Planning should have failed but did not.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Tests that negative blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with negative blend_radius +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestNegativeBlendRadius)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadii(0, -1.0);

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, response.error_code.val) << "Planning should have failed but did not.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Tests that overlapping blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with overlapping blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestOverlappingBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.setBlendRadii(0, 10*seq.getBlendRadius(0));

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN, response.error_code.val) << "Incorrect error code";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Planned trajectory not empty.";
}

/**
 * @brief Tests that too large blend radii are detected.
 *
 * Test Sequence:
 *    1. Generate request with too large blend radii +  Call sequence service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestTooLargeBlendRadii)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  seq.erase(2, seq.size());
  seq.setBlendRadii(0, 10*seq.getBlendRadius(seq.size()-2));

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::FAILURE, response.error_code.val) << "Incorrect error code";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Planned trajectory not empty.";
}

/**
 * @brief Tests behavior of service when sequence with invalid second
 * start state is sent.
 *
 *  Test Sequence:
 *    1. Generate request, second goal has an invalid start state set +  Call sequence service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestSecondStartStateNotFirstGoal)
{
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_2_;
  req_list.items[1].req.start_state.joint_state = testutils::generateJointState({-1., 2., -3., 4., -5., 0.});

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE, response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Planned trajectory should not contain any points.";
}

/**
 * @brief Tests behavior of service when sequence with invalid first goal
 * is sent.
 *
 *  Test Sequence:
 *    1. Generate request with first goal out of workspace +  Call sequence service.
 *    2. Evaluate the result
 *
 *  Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestFirstGoalNotReachable)
{
  pilz_msgs::MotionSequenceRequest req_list = blend_command_list_2_;
  req_list.items[0].req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = 27;

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req_list;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION, response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Trajectory should not contain any points.";
}

/**
 * @brief Tests that incorrect link_names are detected.
 *
 * Test Sequence:
 *    1. Create sequence and send it.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command fails, result trajectory is empty.
 */
TEST_F(IntegrationTestSequenceService, TestInvalidLinkName)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  seq.setAllBlendRadiiToZero();

  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  // Invalidate link name
  req.items.at(1).req.goal_constraints.at(0).position_constraints.at(0).link_name = "InvalidLinkName";
  req.items.at(1).req.goal_constraints.at(0).orientation_constraints.at(0).link_name = "InvalidLinkName";

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_NE(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Incorrect error code.";
  EXPECT_EQ(0u, response.trajectory.joint_trajectory.points.size()) << "Planned trajectory not empty.";
}

/**
 * @brief Tests the LIN-LIN blending.
 *
 * Test Sequence:
 *    1. Call service.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Error code of the result is success.
 */
TEST_F(IntegrationTestSequenceService, TestLinLinBlending)
{
  for(const auto& test_data : test_data_)
  {
    pilz_msgs::MotionSequenceRequest req_list;
    testutils::generateRequestMsgFromBlendTestData(robot_model_, test_data,
                                                   "LIN", planning_group_,
                                                   target_link_, req_list);

    pilz_msgs::GetMotionSequence srv;
    srv.request.commands = req_list;

    ASSERT_TRUE(client_.call(srv));

    const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
    EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning of blend trajectory failed!";
    EXPECT_GT(response.trajectory.joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
  }
}

/**
 * @brief Tests the execution of a sequence with more than two commands.
 *
 * Test Sequence:
 *    1. Call service with serveral requests.
 *    2. Evaluate the result.
 *
 * Expected Results:
 *    1. MotionPlanResponse is received.
 *    2. Command succeeds, result trajectory is not empty.
 */
TEST_F(IntegrationTestSequenceService, TestLargeRequest)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  // Make copy of sequence commands and add them to the end of sequence.
  // Create large request by making copies of the original sequence commands
  // and adding them to the end of the original sequence.
  size_t N {req.items.size()};
  for(size_t i = 0; i<N; ++i)
  {
    pilz_msgs::MotionSequenceItem item {req.items.at(i)};
    if (i == 0)
    {
      // Remove start state because only the first request
      // is allowed to have a start state in a sequence.
      item.req.start_state = moveit_msgs::RobotState();
    }
    req.items.push_back(item);
  }

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = req;

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Incorrect error code.";
  EXPECT_GT(response.trajectory.joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
}

/**
 * @brief Tests the execution of a sequence command (without blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithoutBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  seq.setAllBlendRadiiToZero();

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Incorrect error code.";
  EXPECT_GT(response.trajectory.joint_trajectory.points.size(), 0u) << "Trajectory should contain points.";
}

/**
 * @brief Tests the execution of a sequence command (with blending)
 * consisting of most of the possible command type combination.
 *
 * Test Sequence:
 *    1. Create sequence goal and send it via ActionClient.
 *    2. Wait for successful completion of command.
 *
 * Expected Results:
 *    1. -
 *    2. ActionClient reports successful completion of command.
 */
TEST_F(IntegrationTestSequenceService, TestComplexSequenceWithBlending)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};

  pilz_msgs::GetMotionSequence srv;
  srv.request.commands = seq.toRequest();

  ASSERT_TRUE(client_.call(srv));

  const moveit_msgs::MotionPlanResponse& response {srv.response.plan_response};
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
