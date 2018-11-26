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

#include "pilz_trajectory_generation/trajectory_generator_circ.h"
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
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string CARTESIAN_POSITION_TOLERANCE("cartesian_position_tolerance");
const std::string CARTESIAN_ANGLE_TOLERANCE("cartesian_angle_tolerance");
const std::string ROTATION_AXIS_NORM_TOLERANCE("rot_axis_norm_tolerance");
const std::string OTHER_TOLERANCE("other_tolerance");


using namespace pilz;

class TrajectoryGeneratorCIRCTest: public testing::TestWithParam<std::string>
{
protected:

  /**
   * @brief Create test scenario for circ trajectory generator
   *
   */
  virtual void SetUp();

  void checkCircResult(const planning_interface::MotionPlanRequest &req,
                       const planning_interface::MotionPlanResponse& res);

protected:
  // ros stuff
  ros::NodeHandle ph_ {"~"};
  robot_model::RobotModelConstPtr robot_model_ {
    robot_model_loader::RobotModelLoader(GetParam()).getModel()};
  std::unique_ptr<TrajectoryGeneratorCIRC> circ_;
  // test data provider
  std::unique_ptr<pilz_industrial_motion_testutils::TestdataLoader> tdp_;
  // motion plan request director
  pilz_industrial_motion_testutils::MotionPlanRequestDirector req_director_;

  // test parameters from parameter server
  std::string planning_group_, target_link_, test_data_file_name_;
  int random_trial_num_;
  double cartesian_position_tolerance_, cartesian_angle_tolerance_, rot_axis_norm_tolerance_, other_tolerance_;
  LimitsContainer planner_limits_;
};


void TrajectoryGeneratorCIRCTest::SetUp()
{
  // get parameters
  ASSERT_TRUE(ph_.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));
  ASSERT_TRUE(ph_.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph_.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_POSITION_TOLERANCE, cartesian_position_tolerance_));
  ASSERT_TRUE(ph_.getParam(CARTESIAN_ANGLE_TOLERANCE, cartesian_angle_tolerance_));
  ASSERT_TRUE(ph_.getParam(ROTATION_AXIS_NORM_TOLERANCE, rot_axis_norm_tolerance_));
  ASSERT_TRUE(ph_.getParam(OTHER_TOLERANCE, other_tolerance_));

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // load the test data provider
  tdp_.reset(new pilz_industrial_motion_testutils::XmlTestdataLoader{test_data_file_name_});
  ASSERT_NE(nullptr, tdp_) << "Failed to load test data by provider.";

  // create the limits container
  pilz::JointLimitsContainer joint_limits =
      pilz::JointLimitsAggregator::getAggregatedLimits(ph_, robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  // Cartesian limits are chose as such values to ease the manually compute the trajectory
  // TODO move into testdata.xml
  cart_limits.setMaxRotationalVelocity(1*M_PI);
  cart_limits.setMaxTranslationalAcceleration(1*M_PI);
  cart_limits.setMaxTranslationalDeceleration(1*M_PI);
  cart_limits.setMaxTranslationalVelocity(1*M_PI);
  planner_limits_.setJointLimits(joint_limits);
  planner_limits_.setCartesianLimits(cart_limits);

  // initialize the LIN trajectory generator
  circ_.reset(new TrajectoryGeneratorCIRC(robot_model_, planner_limits_));
  ASSERT_NE(nullptr, circ_) << "failed to create CIRC trajectory generator";

}

void TrajectoryGeneratorCIRCTest::checkCircResult(const planning_interface::MotionPlanRequest& req,
                                                  const planning_interface::MotionPlanResponse &res)
{
  // check the trajectory
  EXPECT_NEAR(6.0, res.trajectory_->getWayPointDurationFromStart(res.trajectory_->getWayPointCount()),
              other_tolerance_);

  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_TRUE(testutils::isGoalReached(res.trajectory_->getFirstWayPointPtr()->getRobotModel(),
                                       res_msg.trajectory.joint_trajectory,
                                       req,
                                       other_tolerance_));

  EXPECT_TRUE(testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory,
                                              planner_limits_.getJointLimitContainer()));

  // check the trapezoid velocity profile
  int waypoint_index;
  robot_state::RobotState waypoint_state(res.trajectory_->getFirstWayPointPtr()->getRobotModel());
  Eigen::Affine3d waypoint_pose;
  Eigen::AngleAxisd waypoint_aa;
  double radius = 0.3;
  double alpha = 0;
  double angle_trans = 5.0/6.0*M_PI;
  double angle_rot = 0.5*M_PI;

  // check all waypoints are on the cricle and SLERP
  Eigen::Affine3d start_pose = res.trajectory_->getFirstWayPointPtr()->getFrameTransform(target_link_);
  Eigen::Affine3d goal_pose = res.trajectory_->getLastWayPointPtr()->getFrameTransform(target_link_);
  for(std::size_t i = 0; i < res.trajectory_->getWayPointCount(); ++i )
  {
    waypoint_pose = res.trajectory_->getWayPointPtr(i)->getFrameTransform(target_link_);
    EXPECT_NEAR(sqrt(waypoint_pose(0,3)*waypoint_pose(0,3) + waypoint_pose(1,3)*waypoint_pose(1,3)),
                radius, cartesian_position_tolerance_) << "Trajectory way point is not on the circle.";

    EXPECT_TRUE(testutils::checkSLERP(start_pose,
                                      goal_pose,
                                      waypoint_pose,
                                      rot_axis_norm_tolerance_));
  }

  // way point at 0.5s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 0.5);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans/40.0;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot/40.0, waypoint_aa.angle(), cartesian_angle_tolerance_);

  // way point at 1s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 1);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans/10.0;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot/10.0, waypoint_aa.angle(),cartesian_angle_tolerance_);

  // way point at 3s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 3);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans/2.0;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot/2.0, waypoint_aa.angle(),cartesian_angle_tolerance_);

  // way point at 5s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 5);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans*9.0/10.0;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot*9.0/10.0, waypoint_aa.angle(),cartesian_angle_tolerance_);

  // way point at 5.5s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 5.5);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans*39.0/40.0;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot*39.0/40.0, waypoint_aa.angle(),cartesian_angle_tolerance_);

  // way point at 6s
  waypoint_index = testutils::getWayPointIndex(res.trajectory_, 6);
  waypoint_state = res.trajectory_->getWayPoint(waypoint_index);
  waypoint_pose = waypoint_state.getFrameTransform(target_link_);
  // translation
  alpha = angle_trans;
  EXPECT_NEAR(radius*cos(alpha), waypoint_pose(0,3), cartesian_position_tolerance_);
  EXPECT_NEAR(radius*sin(alpha), waypoint_pose(1,3), cartesian_position_tolerance_);
  EXPECT_NEAR(0.65, waypoint_pose(2,3), cartesian_position_tolerance_);
  // rotation
  waypoint_aa = waypoint_pose.linear();
  EXPECT_NEAR(angle_rot, waypoint_aa.angle(),cartesian_angle_tolerance_);

  for(size_t idx = 0; idx < res.trajectory_->getLastWayPointPtr()->getVariableCount(); ++idx)
  {
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableVelocity(idx), other_tolerance_);
    EXPECT_NEAR(0.0, res.trajectory_->getLastWayPointPtr()->getVariableAcceleration(idx), other_tolerance_);
  }
}

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_CASE_P(InstantiationName, TrajectoryGeneratorCIRCTest, ::testing::Values(
                        PARAM_MODEL_NO_GRIPPER_NAME,
                        PARAM_MODEL_WITH_GRIPPER_NAME
                          ));


/**
 * @brief Construct a TrajectoryGeneratorCirc with no limits given
 */
TEST_P(TrajectoryGeneratorCIRCTest, noLimits)
{
  LimitsContainer planner_limits;
  EXPECT_THROW(TrajectoryGeneratorCIRC(this->robot_model_, planner_limits), TrajectoryGeneratorInvalidLimitsException);
}


/**
 * @brief test invalid motion plan request with non zero start velocity
 */
TEST_P(TrajectoryGeneratorCIRCTest, nonZeroStartVelocity)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd1", circ_cmd)) << "failed to get circ command from test data";
  // construct request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // start state has non-zero velocity
  req.start_state.joint_state.velocity.push_back(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  req.start_state.joint_state.velocity.clear();
}

/**
 * @brief Generate invalid circ with to high vel/acc scaling
 */
TEST_P(TrajectoryGeneratorCIRCTest, toFast)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;

  ROS_ERROR_STREAM("Loading point");

  ASSERT_TRUE(tdp_->getCirc("CIRCCmdToFast", circ_cmd)) << "failed to get circ command from test data";
  // construct request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::PLANNING_FAILED);
}

/**
 * @brief Use three points with a really small distance between to trigger a internal throw from KDL
 */
TEST_P(TrajectoryGeneratorCIRCTest, samePoints)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;

  ROS_ERROR_STREAM("Loading point");

  ASSERT_TRUE(tdp_->getCirc("CIRCCmdAllPointsTheSame", circ_cmd)) << "failed to get circ command from test data";
  // construct request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}


/**
 * @brief test invalid motion plan request with no aux point defined
 */
TEST_P(TrajectoryGeneratorCIRCTest, emptyAux)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd1", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  req.path_constraints.position_constraints.clear();
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with no aux point defined
 */
TEST_P(TrajectoryGeneratorCIRCTest, invalidAuxName)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd1", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  req.path_constraints.name = "";
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with no aux point defined
 */
TEST_P(TrajectoryGeneratorCIRCTest, invalidAuxLinkName)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd1", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  req.path_constraints.position_constraints.front().link_name = "";
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME);
}

/**
 * @brief test the circ planner with wrong center point
 */
TEST_P(TrajectoryGeneratorCIRCTest, wrongCenter)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("CIRCCmd3", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with colinear start/goal/center position
 */
TEST_P(TrajectoryGeneratorCIRCTest, colinearCenter)
{
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("CIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with  colinear start/goal/interim position
 */
TEST_P(TrajectoryGeneratorCIRCTest, colinearInterim)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;
  ASSERT_TRUE(tdp_->getCirc("CIRCCmd5", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with half circle with interim point
 *
 * The request contains start/interim/goal so that
 * start, center (not explicitly given) and goal are colinear
 *
 * Expected: Planning should successfully return.
 */
TEST_P(TrajectoryGeneratorCIRCTest, colinearCenterDueToInterim)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;
  ASSERT_TRUE(tdp_->getCirc("CIRCCmd4", circ_cmd)) << "failed to get circ command from test data";
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief test the circ planner with center point and joint goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, centerPointJointGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief A valid circ request contains a helping point (interim or center), in this test a additional
 * point is defined as an invalid test case
 */
TEST_P(TrajectoryGeneratorCIRCTest, InvalidAdditionalPrimitivePose)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // Contains one pose (interim / center)
  ASSERT_EQ(req.path_constraints.position_constraints.back().constraint_region.primitive_poses.size(), 1u);

  // Define a additional pose here
  geometry_msgs::Pose center_position;
  center_position.position.x = 0.0;
  center_position.position.y = 0.0;
  center_position.position.z = 0.65;
  req.path_constraints.position_constraints.back().constraint_region.primitive_poses.push_back(center_position);


  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief Joint Goals are expected to match the start state in number and joint_names
 * Here an additional joint constraints is "falsely" defined to check for the error.
 */
TEST_P(TrajectoryGeneratorCIRCTest, InvalidExtraJointConstraint)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // Define the additional joint constraint
  moveit_msgs::JointConstraint joint_constraint;
  joint_constraint.joint_name = req.goal_constraints.front().joint_constraints.front().joint_name;
  req.goal_constraints.front().joint_constraints.push_back(joint_constraint); //<-- Additional constraint

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
}


/**
 * @brief test the circ planner with center point and pose goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, CenterPointPoseGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief Set a frame id only on the position constrainst
 */
TEST_P(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdPositionConstraints)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);
  req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}


/**
 * @brief Set a frame id only on the orientation constrainst
 */
TEST_P(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdOrientationConstraints)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);
  req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}


/**
 * @brief Set a frame_id on both position and orientation constraints
 */
TEST_P(TrajectoryGeneratorCIRCTest, CenterPointPoseGoalFrameIdBothConstraints)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);

  // Both set
  req.goal_constraints.front().position_constraints.front().header.frame_id = robot_model_->getModelFrame();
  req.goal_constraints.front().orientation_constraints.front().header.frame_id = robot_model_->getModelFrame();

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with joint goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, InterimPointJointGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd3", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with joint goal and a close to zero velocity of the start state
 *
 * The generator is expected to be robust against a velocity beeing almost zero.
 */
TEST_P(TrajectoryGeneratorCIRCTest, InterimPointJointGoalStartVelNearZero)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd3", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCJointReq(robot_model_, circ_cmd);

  // Set velocity near zero
  req.start_state.joint_state.velocity = std::vector<double>(req.start_state.joint_state.position.size(), 1e-16);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

/**
 * @brief test the circ planner with interim point with pose goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, InterimPointPoseGoal)
{
  // get the test data from xml
  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eINTERMEDIATE;
  ASSERT_TRUE(tdp_->getCirc("ValidCIRCCmd3", circ_cmd)) << "failed to get circ command from test data";
  // construct planning request
  moveit_msgs::MotionPlanRequest req = req_director_.getCIRCCartReq(robot_model_, circ_cmd);

  // empty path constraint
  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
  checkCircResult(req, res);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unittest_trajectory_generator_circ");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
