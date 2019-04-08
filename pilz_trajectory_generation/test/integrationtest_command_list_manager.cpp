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

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/time.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>

#include <tf2_eigen/tf2_eigen.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/sequence.h>
#include <pilz_industrial_motion_testutils/lin.h>

#include "test_utils.h"

#include "pilz_trajectory_generation/command_list_manager.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

const std::string TEST_DATA_FILE_NAME("testdata_file_name");

using testutils::hasStrictlyIncreasingTime;
using namespace pilz_trajectory_generation;
using namespace pilz_industrial_motion_testutils;

class IntegrationTestCommandListManager : public testing::TestWithParam<std::string>
{
protected:
  virtual void SetUp();

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};
  std::shared_ptr<pilz_trajectory_generation::CommandListManager> manager_;
  planning_scene::PlanningScenePtr scene_;

  std::string planning_group_;

  std::unique_ptr<pilz_industrial_motion_testutils::TestdataLoader> data_loader_;
};

void IntegrationTestCommandListManager::SetUp()
{
  // get necessary parameters
  if(!robot_model_)
  {
    FAIL() << "Robot model could not be loaded. Maybe the robot_description(\"" <<GetParam() << "\") is missing.";
  }

  std::string test_data_file_name;
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name));

  // load the test data provider
  data_loader_.reset(new pilz_industrial_motion_testutils::XmlTestdataLoader{test_data_file_name, robot_model_});
  ASSERT_NE(nullptr, data_loader_) << "Failed to load test data by provider.";

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
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  ASSERT_GE(seq.size(), 3u);
  seq.erase(3, seq.size());
  seq.setAllBlendRadiiToZero();

  RobotTrajVec_t res123_vec {manager_->solve(scene_, seq.toRequest())};
  EXPECT_EQ(res123_vec.size(), 1);
  EXPECT_GT(res123_vec.front()->getWayPointCount(), 0u);
  EXPECT_TRUE(hasStrictlyIncreasingTime(res123_vec.front())) << "Time steps not strictly positively increasing";

  ROS_INFO("step 2: only first segment");
  pilz_msgs::MotionSequenceRequest req_1;
  req_1.items.resize(1);
  req_1.items.at(0).req = seq.getCmd(0).toRequest();
  req_1.items.at(0).blend_radius = 0.;
  RobotTrajVec_t res1_vec {manager_->solve(scene_, req_1)};
  EXPECT_EQ(res1_vec.size(), 1);
  EXPECT_GT(res1_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res1_vec.front()->getFirstWayPoint().getVariableCount(),
            req_1.items.at(0).req.start_state.joint_state.name.size());

  ROS_INFO("step 3: only second segment");
  pilz_msgs::MotionSequenceRequest req_2;
  req_2.items.resize(1);
  req_2.items.at(0).req = seq.getCmd(1).toRequest();
  req_2.items.at(0).blend_radius = 0.;
  RobotTrajVec_t res2_vec {manager_->solve(scene_, req_2)};
  EXPECT_EQ(res2_vec.size(), 1);
  EXPECT_GT(res2_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res2_vec.front()->getFirstWayPoint().getVariableCount(),
            req_2.items.at(0).req.start_state.joint_state.name.size());


  ROS_INFO("step 4: only third segment");
  pilz_msgs::MotionSequenceRequest req_3;
  req_3.items.resize(1);
  req_3.items.at(0).req = seq.getCmd(2).toRequest();
  req_3.items.at(0).blend_radius = 0.;
  RobotTrajVec_t res3_vec {manager_->solve(scene_, req_3)};
  EXPECT_EQ(res3_vec.size(), 1);
  EXPECT_GT(res3_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res3_vec.front()->getFirstWayPoint().getVariableCount(),
            req_3.items.at(0).req.start_state.joint_state.name.size());


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
  Sequence seq {data_loader_->getSequence("PtpPtpSequence")};
  seq.setAllBlendRadiiToZero();

  RobotTrajVec_t res_vec {manager_->solve(scene_, seq.toRequest())};
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
  Sequence seq {data_loader_->getSequence("PtpLinSequence")};
  seq.setAllBlendRadiiToZero();

  RobotTrajVec_t res_vec {manager_->solve(scene_, seq.toRequest())};
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
  Sequence seq {data_loader_->getSequence("LinPtpSequence")};
  seq.setAllBlendRadiiToZero();

  RobotTrajVec_t res_vec {manager_->solve(scene_, seq.toRequest())};
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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  RobotTrajVec_t res_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_vec.size(), 1);
  EXPECT_GT(res_vec.front()->getWayPointCount(), 0u);
  EXPECT_EQ(res_vec.front()->getFirstWayPoint().getVariableCount(),
            req.items.at(0).req.start_state.joint_state.name.size());


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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  LinCart& lin {seq.getCmd<LinCart>(0)};
  lin.getGoalConfiguration().getPose().position.y = 2700;
  EXPECT_THROW(manager_->solve(scene_, seq.toRequest()), PlanningPipelineException);
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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  const LinCart& lin {seq.getCmd<LinCart>(0)};
  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  req.items.at(1).req.start_state = lin.getGoalConfiguration().toMoveitMsgsRobotState();
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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  seq.setBlendRadii(0,-0.3);
  EXPECT_THROW(manager_->solve(scene_, seq.toRequest()), NegativeBlendRadiusException);
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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  seq.setBlendRadii(1, 0.03);
  EXPECT_THROW(manager_->solve(scene_, seq.toRequest()), LastBlendRadiusNotZeroException);
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
  Sequence seq {data_loader_->getSequence("TestBlend")};
  seq.setBlendRadii(0, 42.0);
  EXPECT_THROW(manager_->solve(scene_, seq.toRequest()), BlendingFailedException);
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
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  ASSERT_GE(seq.size(), 3u);
  seq.erase(3, seq.size());

  RobotTrajVec_t res_valid_vec {manager_->solve(scene_, seq.toRequest())};
  EXPECT_EQ(res_valid_vec.size(), 1);
  EXPECT_GT(res_valid_vec.front()->getWayPointCount(), 0u);

  // calculate distance from first to second goal
  const PtpJointCart& ptp {seq.getCmd<PtpJointCart>(0)};
  const CircInterimCart& circ {seq.getCmd<CircInterimCart>(1)};
  Eigen::Affine3d p1, p2;
  tf2::fromMsg(ptp.getGoalConfiguration().getPose(), p1);
  tf2::fromMsg(circ.getGoalConfiguration().getPose(), p2);
  auto distance = (p2.translation()-p1.translation()).norm();

  seq.setBlendRadii(1, (distance - seq.getBlendRadius(0) + 0.01) ); // overlapping radii
  EXPECT_THROW(manager_->solve(scene_, seq.toRequest()), OverlappingBlendRadiiException);
}

/**
 * @brief Tests if the planned execution time scales correctly with the number
 * of repetitions.
 *
 * Test Sequence:
 *    1. Generate trajectory and save calculated execution time.
 *    2. Generate request with repeated path along the points from Test Step 1
 *      (repeated two times).
 *
 * Expected Results:
 *    1. Blending succeeds, result trajectory is not empty.
 *    2. Blending succeeds, planned execution time should be approx N times
 *       the single planned execution time.
 */
TEST_P(IntegrationTestCommandListManager, TestExecutionTime)
{
  Sequence seq {data_loader_->getSequence("ComplexSequence")};
  RobotTrajVec_t res_single_vec {manager_->solve(scene_, seq.toRequest())};
  EXPECT_EQ(res_single_vec.size(), 1);
  EXPECT_GT(res_single_vec.front()->getWayPointCount(), 0u);

  pilz_msgs::MotionSequenceRequest req {seq.toRequest()};
  // Create large request by making copies of the original sequence commands
  // and adding them to the end of the original sequence.
  const size_t N {req.items.size()};
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

  RobotTrajVec_t res_n_vec {manager_->solve(scene_, req)};
  EXPECT_EQ(res_n_vec.size(), 1);
  EXPECT_GT(res_n_vec.front()->getWayPointCount(), 0u);

  const double trajectory_time_1 = res_single_vec.front()->getWayPointDurationFromStart(
        res_single_vec.front()->getWayPointCount()-1);
  const double trajectory_time_n = res_n_vec.front()->getWayPointDurationFromStart(
        res_n_vec.front()->getWayPointCount()-1);
  double multiplicator = req.items.size() / N;
  EXPECT_LE(trajectory_time_n, trajectory_time_1*multiplicator);
  EXPECT_GE(trajectory_time_n, trajectory_time_1 * multiplicator * 0.5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_command_list_manager");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
