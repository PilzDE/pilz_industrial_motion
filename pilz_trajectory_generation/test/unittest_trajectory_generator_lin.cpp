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

#include "pilz_trajectory_generation/trajectory_generator_lin.h"
#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "test_utils.h"
#include "pilz_industrial_motion_testutils/xml_testdata_loader.h"
#include "pilz_industrial_motion_testutils/motion_plan_request_director.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>

const std::string PARAM_MODEL_NO_GRIPPER_NAME {"robot_description"};
const std::string PARAM_MODEL_WITH_GRIPPER_NAME {"robot_description_pg70"};

//parameters from parameter server
const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string TARGET_LINK_HCD("target_link_hand_computed_data");
const std::string RANDOM_TEST_TRIAL_NUM("random_trial_number");
const std::string JOINT_POSITION_TOLERANCE("joint_position_tolerance");
const std::string JOINT_VELOCITY_TOLERANCE("joint_velocity_tolerance");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");
const std::string ROTATION_AXIS_NORM_TOLERANCE("rot_axis_norm_tolerance");
const std::string VELOCITY_SCALING_FACTOR("velocity_scaling_factor");
const std::string OTHER_TOLERANCE("other_tolerance");

using namespace pilz;

/**
 * @brief Parameterized unittest of trajectory generator LIN to enable tests against
 * different robot models.The parameter is the name of robot model parameter on the
 * ros parameter server.
 */
class TrajectoryGeneratorLINTest: public testing::TestWithParam<std::string>
{
protected:

  /**
   * @brief Create test scenario for lin trajectory generator
   *
   */
  virtual void SetUp();

  bool checkLinResponse(const planning_interface::MotionPlanRequest& req,
                        const planning_interface::MotionPlanResponse& res);

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};

  // lin trajectory generator using model without gripper
  std::unique_ptr<TrajectoryGenerator> lin_;
  // test data provider
  std::unique_ptr<pilz_industrial_motion_testutils::TestdataLoader> tdp_;
  // motion plan request director
  pilz_industrial_motion_testutils::MotionPlanRequestDirector req_director_;

  // test parameters from parameter server
  std::string planning_group_, target_link_hcd_, test_data_file_name_;
  int random_trial_num_;
  double joint_position_tolerance_, joint_velocity_tolerance_, pose_norm_tolerance_,
  rot_axis_norm_tolerance_, velocity_scaling_factor_, other_tolerance_;
  LimitsContainer planner_limits_;

};



void TrajectoryGeneratorLINTest::SetUp()
{
  // get the parameters
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(TARGET_LINK_HCD, target_link_hcd_));
  ASSERT_TRUE(ph_.getParam(RANDOM_TEST_TRIAL_NUM, random_trial_num_));
  ASSERT_TRUE(ph_.getParam(JOINT_POSITION_TOLERANCE, joint_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(JOINT_VELOCITY_TOLERANCE, joint_velocity_tolerance_));
  ASSERT_TRUE(ph_.getParam(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(ROTATION_AXIS_NORM_TOLERANCE, rot_axis_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(VELOCITY_SCALING_FACTOR, velocity_scaling_factor_));
  ASSERT_TRUE(ph_.getParam(OTHER_TOLERANCE, other_tolerance_));

  testutils::checkRobotModel(robot_model_, planning_group_, target_link_hcd_);

  // load the test data provider
  tdp_.reset(new pilz_industrial_motion_testutils::XmlTestdataLoader{test_data_file_name_});
  ASSERT_NE(nullptr, tdp_) << "Failed to load test data by provider.";

  // create the limits container
  // TODO, move this also into test data set
  pilz::JointLimitsContainer joint_limits =
      pilz::JointLimitsAggregator::getAggregatedLimits(ph_, robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  cart_limits.setMaxRotationalVelocity(0.5*M_PI);
  cart_limits.setMaxTranslationalAcceleration(2);
  cart_limits.setMaxTranslationalDeceleration(2);
  cart_limits.setMaxTranslationalVelocity(1);
  planner_limits_.setJointLimits(joint_limits);
  planner_limits_.setCartesianLimits(cart_limits);

  // initialize the LIN trajectory generator
  lin_.reset(new TrajectoryGeneratorLIN(robot_model_, planner_limits_));
  ASSERT_NE(nullptr, lin_) << "Failed to create LIN trajectory generator.";
}

bool TrajectoryGeneratorLINTest::checkLinResponse(const planning_interface::MotionPlanRequest& req,
                                                  const planning_interface::MotionPlanResponse& res)
{
  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  if(!testutils::isGoalReached(robot_model_,
                               res_msg.trajectory.joint_trajectory,
                               req,
                               pose_norm_tolerance_))
  {
    return false;
  }

  if(!testutils::checkCartesianLinearity(robot_model_,
                                         res_msg.trajectory.joint_trajectory,
                                         req,
                                         pose_norm_tolerance_,
                                         rot_axis_norm_tolerance_))
  {
    return false;
  }

  if(!testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory,
                                      planner_limits_.getJointLimitContainer()))
  {
    return false;
  }

  return true;
}


// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P(InstantiationName, TrajectoryGeneratorLINTest, ::testing::Values(
                          PARAM_MODEL_NO_GRIPPER_NAME,
                          PARAM_MODEL_WITH_GRIPPER_NAME
                          ));

/**
 * @brief test the lin planner with invalid motion plan request which has non zero start velocity
 */
TEST_P(TrajectoryGeneratorLINTest, nonZeroStartVelocity)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd1", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest req = req_director_.getLINJointReq(robot_model_,
                                                                    lin_cmd);

  // add non-zero velocity in the start state
  req.start_state.joint_state.velocity.push_back(1.0);

  // try to generate the result
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(lin_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
}

/**
 * @brief test the lin planner with joint space goal
 */
TEST_P(TrajectoryGeneratorLINTest, jointSpaceGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd1", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with joint space goal with start velocity almost zero
 */
TEST_P(TrajectoryGeneratorLINTest, jointSpaceGoalNearZeroStartVelocity)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd1", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  // Set velocity near zero
  lin_joint_req.start_state.joint_state.velocity
    = std::vector<double>(lin_joint_req.start_state.joint_state.position.size(), 1e-16);

  ROS_ERROR_STREAM(lin_joint_req);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief test the lin planner with Cartesian goal
 */
TEST_P(TrajectoryGeneratorLINTest, cartesianSpaceGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd1", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_cart_req = req_director_.getLINCartReq(robot_model_, lin_cmd);

  // generate lin trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(lin_cart_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_cart_req, res));
}

/**
 * @brief test the trapezoid shape of the planning trajectory in Cartesian space
 */
TEST_P(TrajectoryGeneratorLINTest, cartesianTrapezoidProfile)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd2", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  /// +++++++++++++++++++++++
  /// + plan LIN trajectory +
  /// +++++++++++++++++++++++
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(lin_joint_req, res, 0.01));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  EXPECT_NEAR(2.0, res.trajectory_->getWayPointDurationFromStart(res.trajectory_->getWayPointCount()),
              other_tolerance_);

  /// ++++++++++++++++++++++++++++++
  /// + check the trapzoid profile +
  /// ++++++++++++++++++++++++++++++

  // variables to find the way point at given time
  int waypoint_index;
  robot_state::RobotState waypoint_state(robot_model_);
  Eigen::Affine3d waypoint_pose;
  Eigen::AngleAxisd waypoint_aa;

  // way point at 0.25s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 0.25);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_hcd_);
  // translation
  EXPECT_NEAR(-0.4+0.2/24, waypoint_pose(0,3), other_tolerance_);
  EXPECT_NEAR(-0.2+0.2/24, waypoint_pose(1,3), other_tolerance_);
  EXPECT_NEAR(0.7-0.1/24, waypoint_pose(2,3), other_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR((0.1+0.15/24)*M_PI, waypoint_aa.angle(),other_tolerance_);

  // way point at 0.5s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 0.5);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_hcd_);
  // translation
  EXPECT_NEAR(-0.4+0.2/6, waypoint_pose(0,3), other_tolerance_);
  EXPECT_NEAR(-0.2+0.2/6, waypoint_pose(1,3), other_tolerance_);
  EXPECT_NEAR(0.7-0.1/6, waypoint_pose(2,3), other_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR((0.1+0.15/6)*M_PI, waypoint_aa.angle(),other_tolerance_);

  // way point at 1.0s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 1.0);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_hcd_);
  // translation
  EXPECT_NEAR(-0.4+0.2/2, waypoint_pose(0,3), other_tolerance_);
  EXPECT_NEAR(-0.2+0.2/2, waypoint_pose(1,3), other_tolerance_);
  EXPECT_NEAR(0.7-0.1/2, waypoint_pose(2,3), other_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR((0.1+0.15/2)*M_PI, waypoint_aa.angle(),other_tolerance_);

  // way point at 1.5s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 1.5);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_hcd_);
  // translation
  EXPECT_NEAR(-0.4+0.2*5/6, waypoint_pose(0,3), other_tolerance_);
  EXPECT_NEAR(-0.2+0.2*5/6, waypoint_pose(1,3), other_tolerance_);
  EXPECT_NEAR(0.7-0.1*5/6, waypoint_pose(2,3), other_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR((0.1+0.15*5/6)*M_PI, waypoint_aa.angle(),other_tolerance_);

  // way point at 1.75s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 1.75);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_hcd_);
  // translation
  EXPECT_NEAR(-0.4+0.2*23/24, waypoint_pose(0,3), other_tolerance_);
  EXPECT_NEAR(-0.2+0.2*23/24, waypoint_pose(1,3), other_tolerance_);
  EXPECT_NEAR(0.7-0.1*23/24, waypoint_pose(2,3), other_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR((0.1+0.15*23/24)*M_PI, waypoint_aa.angle(),other_tolerance_);

  // check last point for vel=acc=0
  for(size_t idx = 0; idx < res.trajectory_->getLastWayPointPtr()->getVariableCount(); ++idx)
  {
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableVelocity(idx), other_tolerance_);
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableAcceleration(idx), other_tolerance_);
  }

}

/**
 * @brief Check that lin planner returns 'false' if
 * calculated lin trajectory violates velocity/acceleration or deceleration limits.
 *
 *
 * Test Sequence:
 *    1. Call function with lin request violating velocity/acceleration or deceleration limits.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryGeneratorLINTest, LinPlannerLimitViolation)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmdLimitViolation", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(lin_->generate(lin_joint_req, res));
}

/**
 * @brief test joint linear movement with equal goal and start
 *
 * Test Sequence:
 *    1. Call function with lin request start = goal
 *
 * Expected Results:
 *    1. trajectory generation is successful.
 */
TEST_P(TrajectoryGeneratorLINTest, LinStartEqualsGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINStartEqualsGoal", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(lin_->generate(lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS);

  // check the resulted trajectory
  EXPECT_TRUE(checkLinResponse(lin_joint_req, res));
}

/**
 * @brief Checks that constructor throws an exception if no limits are given.
 *
 * Test Sequence:
 *    1. Call Ctor without set limits.
 *
 * Expected Results:
 *    1. Ctor throws exception.
 */
TEST_P(TrajectoryGeneratorLINTest, CtorNoLimits)
{
  pilz::LimitsContainer planner_limits;

  EXPECT_THROW(pilz::TrajectoryGeneratorLIN(robot_model_, planner_limits),
               pilz::TrajectoryGeneratorInvalidLimitsException);
}

/**
 * @brief Checks that generate() function returns 'false' if called with an
 * incorrect number of joints.
 *
 * Test Sequence:
 *    1. Call functions with incorrect number of joints.
 *
 * Expected Results:
 *    1. Function returns 'false'.
 */
TEST_P(TrajectoryGeneratorLINTest, IncorrectJointNumber)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  ASSERT_TRUE(tdp_->getLin("LINCmd1", lin_cmd));

  // construct motion plan request
  moveit_msgs::MotionPlanRequest lin_joint_req = req_director_.getLINJointReq(robot_model_, lin_cmd);

  // Ensure that request consists of an incorrect number of joints.
  lin_joint_req.goal_constraints.front().joint_constraints.pop_back();

  // generate the LIN trajectory
  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(lin_->generate(lin_joint_req, res));
  EXPECT_TRUE(res.error_code_.val == moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_trajectory_generator_lin");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
