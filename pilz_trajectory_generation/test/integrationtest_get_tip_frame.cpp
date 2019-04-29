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
#include <stdexcept>
#include <vector>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/kinematics_base/kinematics_base.h>

#include "pilz_trajectory_generation/tip_frame_getter.h"

using ::testing::_;

using namespace kinematics;

class KinematicsBaseMock : public KinematicsBase
{
public:
  MOCK_CONST_METHOD0(getTipFrames, const std::vector<std::string>&());

  MOCK_CONST_METHOD5(getPositionIK, bool(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                         std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                         const kinematics::KinematicsQueryOptions& options));

  MOCK_CONST_METHOD6(searchPositionIK, bool(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options));

  MOCK_CONST_METHOD7(searchPositionIK, bool(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options));

  MOCK_CONST_METHOD7(searchPositionIK, bool(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) );

  MOCK_CONST_METHOD8(searchPositionIK, bool(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                            const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) );

  MOCK_CONST_METHOD3(getPositionFK, bool(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                         std::vector<geometry_msgs::Pose>& poses) );

  MOCK_CONST_METHOD0(getJointNames, const std::vector<std::string>&());
  MOCK_CONST_METHOD0(getLinkNames, const std::vector<std::string>&());
};

class PilzModbusClientMock : public moveit::core::JointModelGroup
{
public:
  MOCK_METHOD0(getSolverInstance, const kinematics::KinematicsBaseConstPtr());

};


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
  {
    std::shared_ptr<NoSolverException> nse_ex {new NoSolverException("")};
    EXPECT_EQ(nse_ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }

  {
    std::shared_ptr<MoreThanOneTipFrameException> ex {new MoreThanOneTipFrameException("")};
    EXPECT_EQ(ex->getErrorCode(), moveit_msgs::MoveItErrorCodes::FAILURE);
  }
}

/**
 * @brief Checks that an exceptions is thrown in case a group does not
 * possess a solver.
 */
TEST_F(IntegrationGetTipFrame, TestExceptionNoSolver)
{
  EXPECT_THROW(getSolverTipFrame(robot_model_->getJointModelGroup("fake_group")), NoSolverException);
}

/**
 * @brief Checks that an exceptions is thrown in case a nullptr is
 * specified as JointModelGroup.
 */
TEST_F(IntegrationGetTipFrame, NullptrJointGroup)
{
  EXPECT_THROW(hasSolver(nullptr), std::invalid_argument);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "integrationtest_get_tip_frame");
  testing::InitGoogleTest(&argc, argv);

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
