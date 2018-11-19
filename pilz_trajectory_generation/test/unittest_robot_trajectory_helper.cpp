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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <pilz_trajectory_generation/robot_trajectory_helper.h>
#include <pilz_trajectory_generation/trajectory_functions.h>
#include "test_utils.h"

const std::string PARAM_MODEL_NAME {"robot_description"};
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");

class RobotTrajectoryHelperTest : public testing::Test
{
protected:
  /**
  * @brief Set up the robot model.
  */
  virtual void SetUp();

protected:
  ros::NodeHandle ph_ {"~"};
  std::string planning_group_, target_link_;
  robot_model_loader::RobotModelLoader robot_model_loader_ {PARAM_MODEL_NAME};
  robot_model::RobotModelConstPtr robot_model_ {robot_model_loader_.getModel()};
};

void RobotTrajectoryHelperTest::SetUp()
{
  // get parameters from parameter server
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));

  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);
}

/**
 * @brief Test appending a trajectory whose first state gets redundant.
 *
 * Test Sequence:
 *  1. Create two trajectories where the last point of the first is equal to the first point of the second.
 *  2. Call appendWithoutRedundantFirstState.
 *
 * Expected Results:
 *  1. -
 *  2. The trajectory is appended without the first point.
 */
TEST_F(RobotTrajectoryHelperTest, testAppendWithoutRedundantFirstState)
{
  // Step 1
  double duration_from_previous = 0.1;
  std::vector<double> zeros;
  zeros.resize(robot_model_->getVariableCount(), 0.0);

  moveit::core::RobotStatePtr robot_state1 = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state1->setJointGroupPositions(planning_group_, zeros);
  robot_state1->setJointGroupVelocities(planning_group_, zeros);
  robot_state1->setJointGroupAccelerations(planning_group_, zeros);

  moveit::core::RobotStatePtr robot_state2 = std::make_shared<moveit::core::RobotState>(*robot_state1);
  robot_state2->setVariablePosition(0, 0.1);

  robot_trajectory::RobotTrajectoryPtr traj1 = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_,
                                                                                                   planning_group_);
  traj1->addSuffixWayPoint(robot_state1, duration_from_previous);
  traj1->addSuffixWayPoint(robot_state2, duration_from_previous);

  robot_trajectory::RobotTrajectoryPtr traj2 = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_,
                                                                                                   planning_group_);

  traj2->addSuffixWayPoint(robot_state2, duration_from_previous);
  traj2->addSuffixWayPoint(robot_state1, duration_from_previous);

  // Step 2
  pilz_trajectory_generation::RobotTrajectoryHelper::appendWithoutRedundantFirstState(traj1, *traj2);

  ASSERT_EQ(3u, traj1->getWayPointCount());

  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state1, traj1->getWayPointPtr(0), planning_group_, 1e-12));
  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state2, traj1->getWayPointPtr(1), planning_group_, 1e-12));
  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state1, traj1->getWayPointPtr(2), planning_group_, 1e-12));

  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(0));
  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(1));
  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(2));
}

/**
 * @brief Test appending a trajectory whose first state is different.
 *
 * Test Sequence:
 *  1. Create two trajectories where the last point of the first is different to the first point of the second.
 *  2. Call appendWithoutRedundantFirstState.
 *
 * Expected Results:
 *  1. -
 *  2. The whole trajectory is appended.
 */
TEST_F(RobotTrajectoryHelperTest, testAppendWithoutRedundantFirstStateDifferentFirstState)
{
  // Step 1
  double duration_from_previous = 0.1;
  std::vector<double> zeros;
  zeros.resize(robot_model_->getVariableCount(), 0.0);

  moveit::core::RobotStatePtr robot_state1 = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state1->setJointGroupPositions(planning_group_, zeros);
  robot_state1->setJointGroupVelocities(planning_group_, zeros);
  robot_state1->setJointGroupAccelerations(planning_group_, zeros);

  moveit::core::RobotStatePtr robot_state2 = std::make_shared<moveit::core::RobotState>(*robot_state1);
  robot_state2->setVariablePosition(0, 0.1);

  robot_trajectory::RobotTrajectoryPtr traj1 = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_,
                                                                                                   planning_group_);
  traj1->addSuffixWayPoint(robot_state1, duration_from_previous);
  traj1->addSuffixWayPoint(robot_state2, duration_from_previous);

  robot_trajectory::RobotTrajectoryPtr traj2 = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_,
                                                                                                   planning_group_);

  traj2->addSuffixWayPoint(robot_state1, duration_from_previous);
  traj2->addSuffixWayPoint(robot_state2, duration_from_previous);

  // Step 2
  pilz_trajectory_generation::RobotTrajectoryHelper::appendWithoutRedundantFirstState(traj1, *traj2);

  ASSERT_EQ(4u, traj1->getWayPointCount());

  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state1, traj1->getWayPointPtr(0), planning_group_, 1e-12));
  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state2, traj1->getWayPointPtr(1), planning_group_, 1e-12));
  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state1, traj1->getWayPointPtr(2), planning_group_, 1e-12));
  ASSERT_TRUE(pilz::isRobotStateEqual(robot_state2, traj1->getWayPointPtr(3), planning_group_, 1e-12));

  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(0));
  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(1));
  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(2));
  ASSERT_EQ(duration_from_previous, traj1->getWayPointDurationFromPrevious(3));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_robot_trajectory_helper");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}