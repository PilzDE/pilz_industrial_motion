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
#include <gmock/gmock.h>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <ros/ros.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <pilz_testutils/async_test.h>

#include "pilz_msgs/MoveGroupSequenceAction.h"
#include "test_utils.h"

static constexpr int WAIT_FOR_RESULT_TIME_OUT {5}; //seconds
static constexpr int TIME_BEFORE_CANCEL_GOAL {2}; //seconds
static constexpr int WAIT_FOR_ACTION_SERVER_TIME_OUT {10}; //seconds

const std::string SEQUENCE_ACTION_NAME("/sequence_move_group");
const std::string BLEND_DATA_PREFIX("test_data/");
const std::string BLEND_NE_DATA_PREFIX("ne_test_data/");

// Parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");
const std::string BLEND_DATASET_NUM("blend_dataset_num");
const std::string BLEND_NE_DATASET_NUM("blend_ne_dataset_num");

// events for callback tests
const std::string GOAL_SUCCEEDED_EVENT = "GOAL_SUCCEEDED";
const std::string SERVER_IDLE_EVENT = "SERVER_IDLE";

class IntegrationTestSequenceAction : public testing::Test, public testing::AsyncTest
{
protected:

  virtual void SetUp();

  virtual void TearDown() {}

public:
  MOCK_METHOD0(active_callback, void());
  MOCK_METHOD1(feedback_callback, void(const pilz_msgs::MoveGroupSequenceFeedbackConstPtr& feedback));
  MOCK_METHOD2(done_callback, void(const actionlib::SimpleClientGoalState& state,
                                   const pilz_msgs::MoveGroupSequenceResultConstPtr& result));

protected:
  ros::NodeHandle ph_ {"~"};
  actionlib::SimpleActionClient<pilz_msgs::MoveGroupSequenceAction> ac_blend_{ph_, SEQUENCE_ACTION_NAME, true};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  ros::AsyncSpinner spinner_ {1};

  robot_model_loader::RobotModelLoader model_loader_;
  robot_model::RobotModelPtr robot_model_;
  std::vector<testutils::blend_test_data> test_data_, ne_test_data_;
  std::string planning_group_, target_link_;
  double joint_position_tolerance_;
};

void IntegrationTestSequenceAction::SetUp()
{
  spinner_.start();

  // get necessary parameters
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));

  // create robot model
  robot_model_ = model_loader_.getModel();

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // get test data set
  int blend_dataset_num, blend_ne_dateset_num;
  ASSERT_TRUE(ph_.getParam(BLEND_DATASET_NUM, blend_dataset_num));
  ASSERT_TRUE(ph_.getParam(BLEND_NE_DATASET_NUM, blend_ne_dateset_num));
  ASSERT_TRUE(testutils::getBlendTestData(ph_, blend_dataset_num, BLEND_DATA_PREFIX, test_data_));
  ASSERT_TRUE(testutils::getBlendTestData(ph_, blend_ne_dateset_num, BLEND_NE_DATA_PREFIX, ne_test_data_));

  // wait for action server
  ASSERT_TRUE(ac_blend_.waitForServer(ros::Duration(WAIT_FOR_ACTION_SERVER_TIME_OUT))) << "Action server is not active.";

  // move to default position
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_);
  move_group_->setPlannerId("PTP");
  move_group_->setGoalTolerance(joint_position_tolerance_);
  robot_state::RobotState rstate(robot_model_);
  rstate.setToDefaultValues();
  move_group_->setJointValueTarget(rstate);
  move_group_->move();
  robot_state::RobotStateConstPtr current_state = move_group_->getCurrentState();
  ASSERT_EQ(current_state->getVariableCount(), rstate.getVariableCount());
  for(size_t i=0; i<current_state->getVariableCount(); ++i)
  {
    EXPECT_NEAR(rstate.getVariablePosition(i), current_state->getVariablePosition(i), joint_position_tolerance_)
        << i << "th joint did not reach default position.";
  }
}


/**
 * @brief  Tests the blend action server with empty motion blend list
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal with empty motion blend list
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Empty blend goal is sent to the action server.
 *    3. Error code of the blend result is INVALIDE_MOTION_PLAN.
 */
TEST_F(IntegrationTestSequenceAction, blendEmptyBlendList)
{
  pilz_msgs::MoveGroupSequenceGoal seq_goal;

  // send goal
  ac_blend_.sendGoalAndWait(seq_goal);
  pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
  EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
  EXPECT_EQ(res->planned_trajectory.joint_trajectory.points.size(), 0u)
      << "Planned trajectory is not empty of empty motion blend list.";
}


/**
 * @brief  Tests the blend action server of LIN-LIN blend
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is success.
 */
TEST_F(IntegrationTestSequenceAction, blendLINLIN)
{
  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);
    // send goal
    ac_blend_.sendGoalAndWait(seq_goal);
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS) << "Blend failed.";
    EXPECT_NE(res->planned_trajectory.joint_trajectory.points.size(), 0u)
        << "Planned trajectory is empty.";
  }
}

/**
 * @brief  Tests the blend action server of LIN-LIN blend using invalid (differing) group names
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Invalidate first request (change group_name), send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is failure.
 */
TEST_F(IntegrationTestSequenceAction, blendLINLINInvalidGroupnames)
{
  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);

    // Manipulate the group for negative test
    seq_goal.request.items.at(0).req.group_name = "WrongGroupName";
    // send goal
    ac_blend_.sendGoalAndWait(seq_goal);
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME) << "Blend failed.";
    EXPECT_EQ(res->planned_trajectory.joint_trajectory.points.size(), 0u)
        << "Planned trajectory is empty.";
  }
}

/**
 * @brief  Tests the blend action server of LIN-LIN blend using invalid blend radius
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is not success and the planned trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, blendLINLIN_invalidBlendRadius)
{
  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);
    // set invalid blend radius
    seq_goal.request.items.at(0).blend_radius = -1;
    // send goal
    ac_blend_.sendGoalAndWait(seq_goal);
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) << "Blend failed.";
    EXPECT_EQ(res->planned_trajectory.joint_trajectory.points.size(), 0u)
        << "Planned trajectory is empty.";
  }
}

/**
 * @brief  Tests the blend action server of LIN-LIN blend using invalid test data
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is not success and the planned trajectory is empty.
 */
TEST_F(IntegrationTestSequenceAction, negativeBlendLINLIN)
{
  for(const auto& test_data : ne_test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);
    // send goal
    ac_blend_.sendGoalAndWait(seq_goal);
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_NE(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
    EXPECT_EQ(res->planned_trajectory.joint_trajectory.points.size(), 0u);
  }
}


//*******************************************************
//*** matcher for callback functions of action server ***
//*******************************************************
MATCHER_P(FeedbackStateEq, state, "") { return arg->state == state; }
MATCHER(IsResultSuccess, "") { return arg->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS; }
MATCHER(IsResultNotEmpty, "") { return arg->planned_trajectory.joint_trajectory.points.size() > 0; }

/**
 * @brief  Tests the blend action server of LIN-LIN blend using callbacks
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is success. Active-, feedback- and done-callbacks are called.
 */
TEST_F(IntegrationTestSequenceAction, blendLINLINcb)
{
  using ::testing::_;
  using ::testing::AllOf;
  using ::testing::AtLeast;
  using ::testing::InSequence;

  namespace ph = std::placeholders;

  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);

    // set expectations (no guarantee, that done callback is called before idle feedback)
    EXPECT_CALL(*this, active_callback())
        .Times(1)
        .RetiresOnSaturation();

    EXPECT_CALL(*this, done_callback(_, AllOf(IsResultSuccess(), IsResultNotEmpty())))
        .Times(1)
        .WillOnce(ACTION_OPEN_BARRIER_VOID(GOAL_SUCCEEDED_EVENT))
        .RetiresOnSaturation();

    // the feedbacks are expected in order
    {
      InSequence dummy;

      EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("PLANNING")))
        .Times(AtLeast(1));
      EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("MONITOR")))
        .Times(AtLeast(1));
      EXPECT_CALL(*this, feedback_callback(FeedbackStateEq("IDLE")))
        .Times(AtLeast(1))
        .WillOnce(ACTION_OPEN_BARRIER_VOID(SERVER_IDLE_EVENT))
        .RetiresOnSaturation();
    }

    // send goal using mocked callback methods
    ac_blend_.sendGoal(seq_goal, std::bind(&IntegrationTestSequenceAction::done_callback, this, ph::_1, ph::_2),
                                 std::bind(&IntegrationTestSequenceAction::active_callback, this),
                                 std::bind(&IntegrationTestSequenceAction::feedback_callback, this, ph::_1));

    // wait for the ecpected events
    BARRIER2({GOAL_SUCCEEDED_EVENT, SERVER_IDLE_EVENT});
  }
}


/**
 * @brief  test cancle goal
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Cancel goal before finish.
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Blend goal canceled. Execution stop.
 */
TEST_F(IntegrationTestSequenceAction, cancelGoal)
{
  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);
    // send goal
    ac_blend_.sendGoal(seq_goal);
    // wait for 2 seconds
    ros::Duration(TIME_BEFORE_CANCEL_GOAL).sleep();
    // cancel the goal
    ac_blend_.cancelGoal();
    ac_blend_.waitForResult(ros::Duration(WAIT_FOR_RESULT_TIME_OUT));
    // result
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::PREEMPTED) << "Error code should be preempted.";
  }
}

/**
 * @brief  Tests the blend action server of LIN-LIN blend without execution
 *
 * Test Sequence:
 *    1. Move the robot to start position.
 *    2. Send blend goal for planning and execution.
 *    3. Evaluate the result
 *
 * Expected Results:
 *    1. Robot moved to start position.
 *    2. Blend goal is sent to the action server.
 *    3. Error code of the blend result is success.
 */
TEST_F(IntegrationTestSequenceAction, blendLINLINOnlyPlanning)
{
  for(const auto& test_data : test_data_)
  {
    // move the robot to start position with ptp
    move_group_->setJointValueTarget(test_data.start_position);
    move_group_->move();

    // create request
    pilz_msgs::MoveGroupSequenceGoal seq_goal;
    seq_goal.planning_options.plan_only = true;
    testutils::generateRequestMsgFromBlendTestData(robot_model_,
                                                   test_data,
                                                   "LIN",
                                                   planning_group_,
                                                   target_link_,
                                                   seq_goal.request);
    // send goal
    ac_blend_.sendGoalAndWait(seq_goal);
    pilz_msgs::MoveGroupSequenceResultConstPtr res = ac_blend_.getResult();
    EXPECT_EQ(res->error_code.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
    EXPECT_NE(res->planned_trajectory.joint_trajectory.points.size(), 0u)
        << "Planned trajectory is empty.";

    // check if robot moved after PTP
    robot_state::RobotStateConstPtr current_state = move_group_->getCurrentState();
    ASSERT_GE(current_state->getVariableCount(), test_data.start_position.size());
    for(size_t i=0; i<test_data.start_position.size(); ++i)
    {
      EXPECT_NEAR(test_data.start_position.at(i), current_state->getVariablePosition(i), joint_position_tolerance_)
          << i << "th joint moved during planning only.";
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_sequence_action_capability");
  ros::NodeHandle nh();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
