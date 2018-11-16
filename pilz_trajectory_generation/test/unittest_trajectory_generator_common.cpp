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
#include <boost/core/demangle.hpp>

#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>

#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "pilz_trajectory_generation/joint_limits_container.h"
#include "pilz_trajectory_generation/trajectory_generator_ptp.h"
#include "pilz_trajectory_generation/trajectory_generator_lin.h"
#include "pilz_trajectory_generation/trajectory_generator_circ.h"

#include "test_utils.h"

const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

//parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string PARAM_TARGET_LINK_NAME("target_link");


/**
 * A value type container to combine type and value
 * In the tests types are trajectory generators.
 * value = 0 refers to robot model without gripper
 * value = 1 refers to robot model with gripper
 */
template<typename T, int N>
class ValueTypeContainer
{
public:
  typedef T Type_;
  static const int Value_ = N;
};
template<typename T, int N>
const int ValueTypeContainer<T, N>::Value_;

typedef ValueTypeContainer<pilz::TrajectoryGeneratorPTP, 0> PTP_NO_GRIPPER;
typedef ValueTypeContainer<pilz::TrajectoryGeneratorPTP, 1> PTP_WITH_GRIPPER;
typedef ValueTypeContainer<pilz::TrajectoryGeneratorLIN, 0> LIN_NO_GRIPPER;
typedef ValueTypeContainer<pilz::TrajectoryGeneratorLIN, 1> LIN_WITH_GRIPPER;
typedef ValueTypeContainer<pilz::TrajectoryGeneratorCIRC, 0> CIRC_NO_GRIPPER;
typedef ValueTypeContainer<pilz::TrajectoryGeneratorCIRC, 1> CIRC_WITH_GRIPPER;

typedef ::testing::Types<PTP_NO_GRIPPER, PTP_WITH_GRIPPER, LIN_NO_GRIPPER, LIN_WITH_GRIPPER,
CIRC_NO_GRIPPER, CIRC_WITH_GRIPPER> TrajectoryGeneratorCommonTestTypes;

typedef ::testing::Types<PTP_NO_GRIPPER, LIN_NO_GRIPPER, CIRC_NO_GRIPPER>
TrajectoryGeneratorCommonTestTypesNoGripper;

typedef ::testing::Types<PTP_WITH_GRIPPER, LIN_WITH_GRIPPER, CIRC_WITH_GRIPPER>
TrajectoryGeneratorCommonTestTypesWithGripper;

/**
 * type parameterized test fixture
 */
template <typename T>
class TrajectoryGeneratorCommonTest : public ::testing::Test {
protected:

  virtual void SetUp()
  {
    ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
    ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));

    testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

    // create the limits container
    pilz::JointLimitsContainer joint_limits =
        pilz::JointLimitsAggregator::getAggregatedLimits(ph_, robot_model_->getActiveJointModels());
    pilz::CartesianLimit cart_limits;
    cart_limits.setMaxRotationalVelocity(0.5*M_PI);
    cart_limits.setMaxTranslationalAcceleration(2);
    cart_limits.setMaxTranslationalDeceleration(2);
    cart_limits.setMaxTranslationalVelocity(1);
    pilz::LimitsContainer planner_limits;
    planner_limits.setJointLimits(joint_limits);
    planner_limits.setCartesianLimits(cart_limits);

    //create planner instance
    trajectory_generator_ = std::unique_ptr<typename T::Type_>(new typename T::Type_(robot_model_, planner_limits));
    ASSERT_NE(nullptr, trajectory_generator_) << "failed to create trajectory generator";

    // create a valid motion plan request with goal in joint space as basis for tests
    req_.group_name = planning_group_;
    req_.max_velocity_scaling_factor = 1.0;
    req_.max_acceleration_scaling_factor = 1.0;
    robot_state::RobotState rstate(robot_model_);
    rstate.setToDefaultValues();
    rstate.setJointGroupPositions(planning_group_, {0,M_PI/2,0,M_PI/2,0,0});
    rstate.setVariableVelocities(std::vector<double>(rstate.getVariableCount(), 0.0));
    moveit::core::robotStateToRobotStateMsg(rstate,req_.start_state,false);
    moveit_msgs::Constraints goal_constraint;
    moveit_msgs::JointConstraint joint_constraint;
    joint_constraint.joint_name = this->robot_model_->getActiveJointModels().front()->getName();
    joint_constraint.position = 0.5;
    goal_constraint.joint_constraints.push_back(joint_constraint);
    req_.goal_constraints.push_back(goal_constraint);
  }

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(!T::Value_ ? PARAM_MODEL_NO_GRIPPER_NAME: PARAM_MODEL_WITH_GRIPPER_NAME).getModel()};

  // trajectory generator
  std::unique_ptr<pilz::TrajectoryGenerator> trajectory_generator_;
  planning_interface::MotionPlanResponse res_;
  planning_interface::MotionPlanRequest req_;
  // test parameters from parameter server
  std::string planning_group_, target_link_;
};
// Define the types we need to test
TYPED_TEST_CASE(TrajectoryGeneratorCommonTest, TrajectoryGeneratorCommonTestTypes);

template <typename T>
class TrajectoryGeneratorCommonTestNoGripper : public TrajectoryGeneratorCommonTest<T> {};
TYPED_TEST_CASE(TrajectoryGeneratorCommonTestNoGripper, TrajectoryGeneratorCommonTestTypesNoGripper);

template <typename T>
class TrajectoryGeneratorCommonTestWithGripper : public TrajectoryGeneratorCommonTest<T> {};
TYPED_TEST_CASE(TrajectoryGeneratorCommonTestWithGripper, TrajectoryGeneratorCommonTestTypesWithGripper);



/**
 * @brief test invalid scaling factor. The scaling factor must be in the range of [0.0001, 1]
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InvalideScalingFactor)
{
  this->req_.max_velocity_scaling_factor = 2.0;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  this->req_.max_velocity_scaling_factor = 1.0;
  this->req_.max_acceleration_scaling_factor = 0;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  this->req_.max_velocity_scaling_factor = 0.00001;
  this->req_.max_acceleration_scaling_factor = 1;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);

  this->req_.max_velocity_scaling_factor = 1;
  this->req_.max_acceleration_scaling_factor = -1;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief Test invalid motion plan request for all trajectory generators
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InvalidGroupName)
{
  this->req_.group_name = "foot";
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME, this->res_.error_code_.val);
}

/**
 * @brief Test invalid motion plan request for all trajectory generators
 */
TYPED_TEST(TrajectoryGeneratorCommonTestNoGripper, GripperGroup)
{
  this->req_.group_name = "gripper";
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME, this->res_.error_code_.val);
}

/**
 * @brief Test invalid motion plan request for all trajectory generators
 */
TYPED_TEST(TrajectoryGeneratorCommonTestWithGripper, GripperGroup)
{
  this->req_.group_name = "gripper";
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
  EXPECT_EQ(moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS, this->res_.error_code_.val);
}

/**
 * @brief Test if there is a valid inverse kinematics solver for this planning group
 * You can only test this case by commenting the planning_context.launch in the .test file
 * //TODO create a separate robot model without ik solver and use it to create a trajectory generator
 */
//TYPED_TEST(TrajectoryGeneratorCommonTest, NoIKSolver)
//{
//  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_, this->res_));
//  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME);
//}

/**
 * @brief test the case of empty joint names in start state
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, EmptyJointNamesInStartState)
{
  this->req_.start_state.joint_state.name.clear();
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief size of joint name and joint position does not match in start state
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InconsistentStartState)
{
  this->req_.start_state.joint_state.name.push_back("joint_7");
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief joint position out of limit in start state
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, StartPostionOutOfLimit)
{
  this->req_.start_state.joint_state.position[0] = 100;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief joint velocity none zero
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, StartPositionVelocityNoneZero)
{
  this->req_.start_state.joint_state.velocity[0] = 100;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief goal constraints is empty
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, EmptyGoalConstraints)
{
  this->req_.goal_constraints.clear();
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief multiple goals
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, MultipleGoals)
{
  moveit_msgs::JointConstraint joint_constraint;
  moveit_msgs::PositionConstraint position_constraint;
  moveit_msgs::OrientationConstraint orientation_constraint;
  moveit_msgs::Constraints goal_constraint;

  // two goal constraints
  this->req_.goal_constraints.push_back(goal_constraint);
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);


  // one joint constraint and one orientation constraint
  goal_constraint.joint_constraints.push_back(joint_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

  // one joint constraint and one Cartesian constraint
  goal_constraint.position_constraints.push_back(position_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

  // two Cartesian constraints
  goal_constraint.joint_constraints.clear();
  goal_constraint.position_constraints.push_back(position_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  goal_constraint.position_constraints.push_back(position_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief invalid joint name in joint constraint
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InvalideJointNameInGoal)
{
  moveit_msgs::JointConstraint joint_constraint;
  joint_constraint.joint_name = "test_joint_2";
  this->req_.goal_constraints.front().joint_constraints[0] = joint_constraint;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief MissingJointConstraint
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, MissingJointConstraint)
{
  moveit_msgs::JointConstraint joint_constraint;
  joint_constraint.joint_name = "test_joint_2";
  this->req_.goal_constraints.front().joint_constraints.pop_back(); //<-- Missing joint constraint
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief invalid joint position in joint constraint
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InvalideJointPositionInGoal)
{
  this->req_.goal_constraints.front().joint_constraints[0].position = 100;
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

/**
 * @brief invalid link name in Cartesian goal constraint
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, InvalidLinkNameInCartesianGoal)
{
  moveit_msgs::PositionConstraint position_constraint;
  moveit_msgs::OrientationConstraint orientation_constraint;
  moveit_msgs::Constraints goal_constraint;
  // link name not set
  goal_constraint.position_constraints.push_back(position_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

  // different link names in position and orientation goals
  goal_constraint.position_constraints.front().link_name = "test_link_1";
  goal_constraint.orientation_constraints.front().link_name = "test_link_2";
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);

  // no solver for the link
  goal_constraint.orientation_constraints.front().link_name = "test_link_1";
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION);
}

/**
 * @brief no pose set in position constraint
 */
TYPED_TEST(TrajectoryGeneratorCommonTest, EmptyPrimitivePoses)
{
  moveit_msgs::PositionConstraint position_constraint;
  moveit_msgs::OrientationConstraint orientation_constraint;
  moveit_msgs::Constraints goal_constraint;
  position_constraint.link_name =
      this->robot_model_->getJointModelGroup(this->planning_group_)->getLinkModelNames().back();
  orientation_constraint.link_name = position_constraint.link_name;

  goal_constraint.position_constraints.push_back(position_constraint);
  goal_constraint.orientation_constraints.push_back(orientation_constraint);
  this->req_.goal_constraints.clear();
  this->req_.goal_constraints.push_back(goal_constraint);
  EXPECT_FALSE(this->trajectory_generator_->generate(this->req_,this->res_));
  EXPECT_EQ(this->res_.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_trajectory_generator_common");
  //ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
