/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <pilz_trajectory_generation/trajectory_blender_transition_window.h>
#include <pilz_trajectory_generation/plan_components_builder.h>

const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string ROBOT_DESCRIPTION_STR {"robot_description"};
const std::string EMPTY_VALUE {""};

using namespace pilz;
using namespace pilz_trajectory_generation;

class IntegrationTestPlanComponentBuilder : public testing::TestWithParam<std::string>
{
protected:
  virtual void SetUp();

protected:
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_STR).getModel() };

  std::string planning_group_;
};

void IntegrationTestPlanComponentBuilder::SetUp()
{
  if(!robot_model_)
  {
    FAIL() << "Robot model could not be loaded. Maybe the robot_description(\"" <<GetParam() << "\") is missing.";
  }

  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P( InstantiationName, IntegrationTestPlanComponentBuilder,
                         ::testing::Values(EMPTY_VALUE) );

/**
 * @brief Checks that each derived MoveItErrorCodeException contains the correct
 * error code.
 *
 */
TEST_P(IntegrationTestPlanComponentBuilder, TestExceptionErrorCodeMapping)
{
  NoBlenderSetException nbs_ex("");
  EXPECT_EQ(nbs_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  NoTipFrameFunctionSetException ntffse_ex("");
  EXPECT_EQ(ntffse_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  NoRobotModelSetException nrms_ex("");
  EXPECT_EQ(nrms_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  BlendingFailedException bf_ex("");
  EXPECT_EQ(bf_ex.getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
}

/**
 * @brief Checks that exception is thrown if no robot model is set.
 *
 */
TEST_P(IntegrationTestPlanComponentBuilder, TestModelSet)
{
  robot_trajectory::RobotTrajectoryPtr traj {new robot_trajectory::RobotTrajectory(robot_model_, planning_group_)};
  PlanComponentsBuilder builder;

  EXPECT_THROW(builder.append(traj, 1.0), NoRobotModelSetException);
}

/**
 * @brief Checks that exception is thrown if no blender is set.
 *
 */
TEST_P(IntegrationTestPlanComponentBuilder, TestNoBlenderSet)
{
  robot_trajectory::RobotTrajectoryPtr traj {new robot_trajectory::RobotTrajectory(robot_model_, planning_group_)};
  PlanComponentsBuilder builder;
  builder.setModel(robot_model_);

  builder.append(traj, 0.0);

  EXPECT_THROW(builder.append(traj, 1.0), NoBlenderSetException);
}

/**
 * @brief Checks that exception is thrown if no tip frame getter function
 * is set.
 *
 */
TEST_P(IntegrationTestPlanComponentBuilder, TestNoTipFrameFuncSet)
{
  robot_trajectory::RobotTrajectoryPtr traj {new robot_trajectory::RobotTrajectory(robot_model_, planning_group_)};
  PlanComponentsBuilder builder;
  builder.setModel(robot_model_);
  LimitsContainer limits;
  std::unique_ptr<TrajectoryBlenderTransitionWindow> blender {new TrajectoryBlenderTransitionWindow(limits)};
  builder.setBlender(std::move(blender));

  builder.append(traj, 0.0);

  EXPECT_THROW(builder.append(traj, 1.0), NoTipFrameFunctionSetException);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_plan_components_builder");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}