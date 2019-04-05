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

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_trajectory_generation/tip_frame_getter.h"

using namespace pilz_trajectory_generation;
class IntegrationGetTipFrame : public testing::Test
{
protected:
  virtual void SetUp();

protected:
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader("robot_description").getModel() };
};

void IntegrationGetTipFrame::SetUp()
{
  if(!robot_model_)
  {
    FAIL() << "Robot model could not be loaded.";
  }
}

TEST_F(IntegrationGetTipFrame, TestExceptionErrorCodeMapping)
{
  std::shared_ptr<EndEffectorException> ee_ex {new EndEffectorException("")};
  EXPECT_EQ(ee_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);

  std::shared_ptr<NoSolverException> nse_ex {new NoSolverException("")};
  EXPECT_EQ(nse_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
}

/**
 * @brief Checks that an exceptions is thrown in case a group does not
 * possess a solver.
 */
TEST_F(IntegrationGetTipFrame, TestExceptionNoSolver)
{
  EXPECT_THROW(getTipFrame(robot_model_->getJointModelGroup("fake_group")), NoSolverException);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_get_tip_frame");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
