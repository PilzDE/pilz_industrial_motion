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

#include <ros/ros.h>
#include <ros/time.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2_eigen/tf2_eigen.h>
#include "test_utils.h"

#include "pilz_trajectory_generation/command_list_manager.h"

#include "motion_plan_request_builder.h"
#include "motion_sequence_request_builder.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");

using testutils::hasStrictlyIncreasingTime;
using namespace pilz_trajectory_generation;

class IntegrationTestCommandListManager : public testing::TestWithParam<std::string>
{
protected:
  virtual void SetUp();

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};
  std::shared_ptr<pilz_trajectory_generation::CommandListManager> manager_ {0};
  planning_scene::PlanningScenePtr scene_ {0};

  std::string planning_group_, target_link_;
  pilz_msgs::MotionSequenceRequest blend_command_lin_lin_;
  pilz_msgs::MotionSequenceRequest blend_command_list_lin_lin_lin_;
  moveit_msgs::MotionPlanRequest  req_lin1_, req_lin2_, req_lin3_;
  moveit_msgs::MotionPlanRequest  req_ptp1_, req_ptp2_;
  geometry_msgs::PoseStamped p1_, p2_, p3_;
};

void IntegrationTestCommandListManager::SetUp()
{
  // get necessary parameters
  if(!robot_model_)
  {
    FAIL() << "Robot model could not be loaded. Maybe the robot_description(\"" <<GetParam() << "\") is missing.";
  }
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  p1_.header.frame_id = robot_model_->getModelFrame();
  p1_.pose.position.x = 0.25;
  p1_.pose.position.y = 0.3;
  p1_.pose.position.z = 0.65;
  p1_.pose.orientation.x = 0.;
  p1_.pose.orientation.y = 0.;
  p1_.pose.orientation.z = 0.;
  p1_.pose.orientation.w = 1.;

  p2_ = p1_;
  p2_.pose.position.x -= 0.15;

  // TODO: move p3 to testUtils
  p3_ = p1_;
  p3_.pose.position.y -= 0.15;

  MotionPlanRequestBuilder builder(planning_group_);

  // goal 1 (with start state) for robot without gripper
  robot_state::RobotState rstate(robot_model_);
  rstate.setToDefaultValues();
  rstate.setJointGroupPositions(planning_group_, {0., 0.007881892504574495, -1.8157263253868452,
                                                  0., 1.8236082178909834, 0.});
  builder.setJointStartState(rstate);
  builder.setGoal(target_link_, p1_);

  req_lin1_ = builder.createLin();
  req_ptp1_ = builder.createPtp();

  // Clear the start state for the next goals
  builder.clearJointStartState();

  // goal 2
  builder.setGoal(target_link_, p2_);
  req_lin2_ = builder.createLin();
  req_ptp2_ = builder.createPtp();

  // goal3
  builder.setGoal(target_link_, p3_);
  req_lin3_ = builder.createLin();

  MotionSequenceRequestBuilder seq_request_builder;
  blend_command_lin_lin_ = seq_request_builder.build({ {req_lin1_, 0.08}, {req_lin2_, 0} });
  blend_command_list_lin_lin_lin_ = seq_request_builder.build({ {req_lin1_, 0.08}, {req_lin2_, 0.05}, {req_lin3_, 0} });

  // Define and set the current scene and manager test object
  scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  manager_ = std::make_shared<pilz_trajectory_generation::CommandListManager>(ph_, robot_model_);
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P(InstantiationName, IntegrationTestCommandListManager, ::testing::Values(
                          PARAM_MODEL_NO_GRIPPER_NAME
                          , PARAM_MODEL_WITH_GRIPPER_NAME
                          ));

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 *
 */
TEST_P(IntegrationTestCommandListManager, TestExceptionErrorCodeMapping)
{
  NegativeBlendRadiusException nbr_ex("");
  EXPECT_EQ(nbr_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  LastBlendRadiusNotZeroException lbrnz_ex("");
  EXPECT_EQ(lbrnz_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  StartStateSetException sss_ex("");
  EXPECT_EQ(sss_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);

  OverlappingBlendRadiiException obr_ex("");
  EXPECT_EQ(obr_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  PlanningPipelineException pp_ex("");
  EXPECT_EQ(pp_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  MultipleEndeffectorException me_ex("");
  EXPECT_EQ(me_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  EndeffectorWithoutLinksException ewl_ex("");
  EXPECT_EQ(ewl_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  NoSolverException ns_ex("");
  EXPECT_EQ(ns_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
}

/**
 * @brief Tests the concatenation of three motion commands.
 *
 * Test Sequence:
 *    1. Generate request with three trajectories and zero blend radius.
 *    2. Generate request with first trajectory and zero blend radius.
 *    3. Generate request with second trajectory and zero blend radius.
 *    4. Generate request with third trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    2. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    3. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *    4. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive
 *       resulting duration in step1 is approximately step2 + step3 + step4
 */
TEST_P(IntegrationTestCommandListManager, concatThreeSegments)
{
  MotionSequenceRequestBuilder seq_request_builder;
  pilz_msgs::MotionSequenceRequest req = seq_request_builder.build({std::make_pair<>(req_lin1_, 0),
                                                                                     std::make_pair<>(req_lin2_, 0),
                                                                                     std::make_pair<>(req_lin3_, 0)});
  RobotTrajVec_t res123_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res123_vec.size(), 1);
  EXPECT_GT(res123_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res123_vec.front())) << "Time steps not strictly positively increasing";

  ROS_INFO("step 2: only first segment");
  req = seq_request_builder.build({std::make_pair<>(req_lin1_, 0)});

  RobotTrajVec_t res1_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res1_vec.size(), 1);
  EXPECT_GT(res1_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res1_vec.front()->getFirstWayPoint().getVariableCount(),
            req.items[0].req.start_state.joint_state.name.size());

  ROS_INFO("step 3: only second segment");
  // Create duplicate of req_lin2_ with start state
  MotionPlanRequestBuilder builder_2(req_lin2_);
  builder_2.setJointStartState(res1_vec.front()->getLastWayPoint());

  moveit_msgs::MotionPlanRequest req2 = builder_2.createLin();
  req = seq_request_builder.build({std::make_pair<>(req2, 0)});
  RobotTrajVec_t res2_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res2_vec.size(), 1);
  EXPECT_GT(res2_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res2_vec.front()->getFirstWayPoint().getVariableCount(),
            req.items[0].req.start_state.joint_state.name.size());


  ROS_INFO("step 4: only third segment");
  // Create duplicate of req_lin3_ with start state
  MotionPlanRequestBuilder builder_3(req_lin3_);
  builder_3.setJointStartState(res2_vec.front()->getLastWayPoint());

  moveit_msgs::MotionPlanRequest req3 = builder_3.createLin();
  req = seq_request_builder.build({std::make_pair<>(req3, 0)});
  RobotTrajVec_t res3_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res3_vec.size(), 1);
  EXPECT_GT(res3_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res3_vec.front()->getFirstWayPoint().getVariableCount(),
            req.items[0].req.start_state.joint_state.name.size());


  // durations for the different segments
  auto t1_2_3 = res123_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount()-1);
  auto t1     = res1_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount()-1);
  auto t2     = res2_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount()-1);
  auto t3     = res3_vec.front()->getWayPointDurationFromStart(res123_vec.front()->getWayPointCount()-1);
  ROS_DEBUG_STREAM("total time: "<< t1_2_3 << " t1:" << t1 << " t2:" << t2 << " t3:" << t3);
  EXPECT_LT(fabs((t1_2_3-t1-t2-t3)), 0.4);
}

/**
 * @brief Tests the concatenation of two ptp commands
 *
 * Test Sequence:
 *    1. Generate request with two PTP trajectories and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_P(IntegrationTestCommandListManager, concatTwoPtpSegments)
{
  MotionSequenceRequestBuilder seq_request_builder;
  pilz_msgs::MotionSequenceRequest req = seq_request_builder.build({ {req_ptp1_, 0},
                                                                     {req_ptp2_, 0} });

  RobotTrajVec_t res_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_vec.size(), 1);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the concatenation of ptp and a lin command
 *
 * Test Sequence:
 *    1. Generate request with PTP and LIN trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_P(IntegrationTestCommandListManager, concatPtpAndLinSegments)
{
  MotionSequenceRequestBuilder seq_request_builder;
  pilz_msgs::MotionSequenceRequest req = seq_request_builder.build({ {req_ptp1_, 0},
                                                                     {req_lin2_, 0} });

  RobotTrajVec_t res_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_vec.size(), 1);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the concatenation of a lin and a ptp command
 *
 * Test Sequence:
 *    1. Generate request with LIN and PTP trajectory and zero blend radius.
 *
 * Expected Results:
 *    1. Generation of concatenated trajectory is successful.
 *       All time steps of resulting trajectory are strictly positive.
 */
TEST_P(IntegrationTestCommandListManager, concatLinAndPtpSegments)
{
  MotionSequenceRequestBuilder seq_request_builder;
  pilz_msgs::MotionSequenceRequest req = seq_request_builder.build({ {req_lin1_, 0},
                                                                     {req_ptp2_, 0} });

  RobotTrajVec_t res_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_vec.size(), 1);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res_vec.front()));
}

/**
 * @brief Tests the blending of motion commands
 *
 *  - Test Sequence:
 *    1. Generate request with two trajectories and request blending.
 *
 *  - Expected Results:
 *    1. blending is successful, result trajectory is not empty
 */
TEST_P(IntegrationTestCommandListManager, blendTwoSegments)
{
  planning_interface::MotionPlanResponse res;
  RobotTrajVec_t res_vec {manager_->solve(scene_, blend_command_lin_lin_)};
  EXPECT_EQ(res_vec.size(), 1);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res_vec.front()->getFirstWayPoint().getVariableCount(),
            blend_command_lin_lin_.items[0].req.start_state.joint_state.name.size());


  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("my_planned_path", 1);
  ros::Duration duration(1.0); // wait to notify possible subscribers
  duration.sleep();

  moveit_msgs::DisplayTrajectory displayTrajectory;
  moveit_msgs::RobotTrajectory rob_traj_msg;
  res_vec.front()->getRobotTrajectoryMsg(rob_traj_msg);
  displayTrajectory.trajectory.push_back(rob_traj_msg);
  pub.publish(displayTrajectory);
}

// ------------------
// FAILURE cases
// ------------------

/**
 * @brief Tests sending an empty blending request.
 *
 * Test Sequence:
 *    1. Generate empty request and request blending.
 *
 * Expected Results:
 *    1. blending is successful, result trajectory container is empty
 */
TEST_P(IntegrationTestCommandListManager, emptyList)
{
  pilz_msgs::MotionSequenceRequest empty_list;
  RobotTrajVec_t res_vec {manager_->solve(scene_, empty_list)};
  EXPECT_TRUE(res_vec.empty());
}

/**
 * @brief Checks that exception is thrown if first goal is not reachable.
 *
 * Test Sequence:
 *    1. Generate request with first goal out of workspace.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, firstGoalNotReachable)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_lin_lin_;
  req.items[0].req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position.y = 27;
  EXPECT_THROW(manager_->solve(scene_, req), PlanningPipelineException);
}

/**
 * @brief Checks that exception is thrown if second goal has a start state.
 *
 * Test Sequence:
 *    1. Generate request, second goal has an invalid start state set.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, startStateNotFirstGoal)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_lin_lin_;
  req.items[1].req.start_state.joint_state = testutils::generateJointState({-1., 2., -3., 4., -5., 0.});
  EXPECT_THROW(manager_->solve(scene_, req), StartStateSetException);
}

/**
 * @brief Checks that exception is thrown in case of blending request with
 * negative blend_radius.
 *
 *  Test Sequence:
 *    1. Generate request, first goal has negative blend_radius.
 *
 *  Expected Results:
 *    1. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, blendingRadiusNegative)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_lin_lin_;
  req.items[0].blend_radius = -0.3;
  EXPECT_THROW(manager_->solve(scene_, req), NegativeBlendRadiusException);
}

/**
 * @brief Checks that exception is thrown if last blend radius is not zero.
 *
 *
 * Test Sequence:
 *    1. Generate request, second goal has non-zero blend_radius.
 *
 * Expected Results:
 *    1. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, lastBlendingRadiusNonZero)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_lin_lin_;
  req.items[1].blend_radius = 0.03;
  EXPECT_THROW(manager_->solve(scene_, req), LastBlendRadiusNotZeroException);
}

/**
 * @brief Checks that exception is thrown if blend radius is greater than the
 * segment.
 *
 * Test Sequence:
 *    1. Generate request with huge blending radius, so that trajectories are
 *       completely inside
 *
 * Expected Results:
 *    2. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, blendRadiusGreaterThanSegment)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_lin_lin_;
  req.items[0].blend_radius = 42.;
  EXPECT_THROW(manager_->solve(scene_, req), BlendingFailedException);
}


/**
 * @brief Checks that exception is thrown if two consecutive blend radii
 * overlap.
 *
 * Test Sequence:
 *    1. Generate request with three trajectories
 *    2. Increase second blend radius, so that the radii overlap
 *
 * Expected Results:
 *    1. blending succeeds, result trajectory is not empty
 *    2. Exception is thrown.
 */
TEST_P(IntegrationTestCommandListManager, blendingRadiusOverlapping)
{
  pilz_msgs::MotionSequenceRequest req = blend_command_list_lin_lin_lin_;

  RobotTrajVec_t res_valid_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_valid_vec.size(), 1);
  EXPECT_GT(res_valid_vec.front()->getWayPointCount(), 0u);

  // calculate distance from first to second goal
  Eigen::Affine3d p1, p2;
  tf2::fromMsg(req.items[0].req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0], p1);
  tf2::fromMsg(req.items[1].req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0], p2);
  auto distance = (p2.translation()-p1.translation()).norm();
  req.items[1].blend_radius = distance - req.items[0].blend_radius + 0.01; // overlapping radii

  EXPECT_THROW(manager_->solve(scene_, req), OverlappingBlendRadiiException);
}

/**
 * @brief
 * Stress test: Planning time for a large number of blending requests
 *
 * Test Sequence:
 *    1. Generate request with three trajectories and measure planning time
 *    2. Generate request with repeated path along the three points
 *
 * Expected Results:
 *    1. blending succeeds, result trajectory is not empty
 *    2. blending succeeds, planning time should be approx N times single planning time
 *       and time from start should be approx. N times single plan
 */
TEST_P(IntegrationTestCommandListManager, largeRequest)
{
  const int n = 30;
  pilz_msgs::MotionSequenceRequest req = blend_command_list_lin_lin_lin_;

  RobotTrajVec_t res_single_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_single_vec.size(), 1);
  EXPECT_GT(res_single_vec.front()->getWayPointCount(), 0u);

  // construct request
  req.items.back().blend_radius = 0.01;
  for(int i = 0; i < n; ++i)
  {
    req.items.push_back(req.items[0]);
    req.items.back().req.start_state.joint_state.position.clear();
    req.items.back().req.start_state.joint_state.velocity.clear();
    req.items.back().req.start_state.joint_state.name.clear();
    req.items.push_back(req.items[1]);
    req.items.push_back(req.items[2]);
  }
  req.items.back().blend_radius = 0.0;

  RobotTrajVec_t res_n_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_n_vec.size(), 1);
  EXPECT_GT(res_n_vec.front()->getWayPointCount(), 0u);

  double trajectory_time_1 = res_single_vec.front()->getWayPointDurationFromStart(
        res_single_vec.front()->getWayPointCount()-1);
  double trajectory_time_n = res_n_vec.front()->getWayPointDurationFromStart(
        res_n_vec.front()->getWayPointCount()-1);
  EXPECT_LE(trajectory_time_n, trajectory_time_1*n);
  EXPECT_GE(trajectory_time_n, trajectory_time_1 * n * 0.5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_command_list_manager");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
