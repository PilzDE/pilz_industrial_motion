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
#include "pilz_industrial_motion_testutils/command_types_typedef.h"
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

#define SKIP_IF_GRIPPER if(GetParam() == PARAM_MODEL_WITH_GRIPPER_NAME) { SUCCEED(); return; };

using namespace pilz;
using namespace pilz_industrial_motion_testutils;

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

  tdp_->setRobotModel(robot_model_);

  // create the limits container
  pilz::JointLimitsContainer joint_limits =
      pilz::JointLimitsAggregator::getAggregatedLimits(ph_, robot_model_->getActiveJointModels());
  CartesianLimit cart_limits;
  // Cartesian limits are chose as such values to ease the manually compute the trajectory

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
  moveit_msgs::MotionPlanResponse res_msg;
  res.getMessage(res_msg);
  EXPECT_TRUE(testutils::isGoalReached(res.trajectory_->getFirstWayPointPtr()->getRobotModel(),
                                       res_msg.trajectory.joint_trajectory,
                                       req,
                                       other_tolerance_));

  EXPECT_TRUE(testutils::checkJointTrajectory(res_msg.trajectory.joint_trajectory,
                                              planner_limits_.getJointLimitContainer()));

  EXPECT_EQ(req.path_constraints.position_constraints.size(),1u);
  EXPECT_EQ(req.path_constraints.position_constraints[0].constraint_region.primitive_poses.size(),1u);

  // Get the circ center
  Eigen::Vector3d circCenter;
  if(req.path_constraints.name == "center")
  {
    tf::pointMsgToEigen(req.path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position, circCenter);
  }
  else if(req.path_constraints.name == "interim")
  {
    Eigen::Vector3d interim;
    tf::pointMsgToEigen(req.path_constraints.position_constraints[0].constraint_region.primitive_poses[0].position, interim);
    Eigen::Vector3d start = res.trajectory_->getFirstWayPointPtr()->getFrameTransform(target_link_).translation();
    Eigen::Vector3d goal = res.trajectory_->getLastWayPointPtr()->getFrameTransform(target_link_).translation();

    const Eigen::Vector3d t = interim - start;
    const Eigen::Vector3d u = goal - start;
    const Eigen::Vector3d v = goal - interim;

    const Eigen::Vector3d w = t.cross(u);

    circCenter = start + (u*t.dot(t)*u.dot(v) - t*u.dot(u)*t.dot(v)) * 0.5/pow(w.norm(),2);
  }

  // Check that all point have the equal distance to the center
  for(std::size_t i = 0; i < res.trajectory_->getWayPointCount(); ++i )
  {
    Eigen::Isometry3d waypoint_pose = res.trajectory_->getWayPointPtr(i)->getFrameTransform(target_link_);
    EXPECT_NEAR((res.trajectory_->getFirstWayPointPtr()->getFrameTransform(target_link_).translation() - circCenter).norm(),
                (circCenter - waypoint_pose.translation()).norm(), cartesian_position_tolerance_);
  }

  std::vector<double> accelerations_transl;
  std::vector<double> accelerations_rot;

  // Iterate over waypoints collect accelerations
  for(size_t i = 2; i < res.trajectory_->getWayPointCount(); ++i)
  {
    auto waypoint_pose_0 = res.trajectory_->getWayPoint(i-2).getFrameTransform(target_link_);
    auto waypoint_pose_1 = res.trajectory_->getWayPoint(i-1).getFrameTransform(target_link_);
    auto waypoint_pose_2 = res.trajectory_->getWayPoint(i).getFrameTransform(target_link_);

    auto t1 = res.trajectory_->getWayPointDurationFromPrevious(i-1);
    auto t2 = res.trajectory_->getWayPointDurationFromPrevious(i);

    //***********************
    // Translational part
    //***********************
    auto vel1 = (waypoint_pose_1.translation() - waypoint_pose_0.translation()).norm() / t1;
    auto vel2 = (waypoint_pose_2.translation() - waypoint_pose_1.translation()).norm() / t2;
    auto acc_transl = (vel2 - vel1) / (t1 + t2);
    accelerations_transl.push_back(acc_transl);

    //***********************
    // Rotational part
    //***********************
    Eigen::AngleAxisd axis_wp12((waypoint_pose_0 * waypoint_pose_1.inverse()).rotation());
    Eigen::AngleAxisd axis_wp23((waypoint_pose_1 * waypoint_pose_2.inverse()).rotation());
    // Check that rotation axis stays the same
    ASSERT_LT((axis_wp12.axis() - axis_wp23.axis()).norm(), 1e-8);

    Eigen::Quaterniond orientation1(waypoint_pose_0.rotation());
    Eigen::Quaterniond orientation2(waypoint_pose_1.rotation());
    Eigen::Quaterniond orientation3(waypoint_pose_2.rotation());

    double angular_vel1 = orientation1.angularDistance(orientation2) / t1;
    double angular_vel2 = orientation2.angularDistance(orientation3) / t2;
    double angular_acc = (angular_vel2 - angular_vel1) / (t1+t2);
    accelerations_rot.push_back(angular_acc);
  }

  //for(auto a : accelerations_rot) std::cout << a << std::endl;

  // Check the accelerations
  ASSERT_TRUE(testutils::hasTrapezoidVelocity(accelerations_transl, 1e-3)); // TODO investigate if this tolerance is acceptable
  ASSERT_TRUE(testutils::hasTrapezoidVelocity(accelerations_rot, 1e-6));

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
  moveit_msgs::MotionPlanRequest req {tdp_->getCircJointCenterCart("circ1_center_2").toRequest()};

  // start state has non-zero velocity
  req.start_state.joint_state.velocity.push_back(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE);
  req.start_state.joint_state.velocity.clear();
}

TEST_P(TrajectoryGeneratorCIRCTest, ValidCommand)
{
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  planning_interface::MotionPlanResponse res;
  EXPECT_TRUE(circ_->generate(circ.toRequest(),res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief Generate invalid circ with to high vel scaling
 */
TEST_P(TrajectoryGeneratorCIRCTest, velScaleToHigh)
{
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  circ.setVelocityScale(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::PLANNING_FAILED);
}

/**
 * @brief Generate invalid circ with to high acc scaling
 */
TEST_P(TrajectoryGeneratorCIRCTest, accScaleToHigh)
{
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  circ.setAccelerationScale(1.0);
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(), res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::PLANNING_FAILED);
}

/**
 * @brief Use three points with a really small distance between to trigger a internal throw from KDL
 */
TEST_P(TrajectoryGeneratorCIRCTest, samePoints)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y += 1e-8;
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.z += 1e-8;
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().getPose().position.x -= 1e-8;
  circ.getGoalConfiguration().getPose().position.y -= 1e-8;
  circ.getGoalConfiguration().getPose().position.z -= 1e-8;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(),res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with no aux point defined
 */
TEST_P(TrajectoryGeneratorCIRCTest, emptyAux)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  // empty path constraint
  planning_interface::MotionPlanRequest req = circ.toRequest();

  req.path_constraints.position_constraints.clear();

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN) << req;
}

/**
 * @brief test invalid motion plan request with no aux name defined
 */
TEST_P(TrajectoryGeneratorCIRCTest, invalidAuxName)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  // empty path constraint
  planning_interface::MotionPlanRequest req = circ.toRequest();

  // empty path constraint
  req.path_constraints.name = "";
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test invalid motion plan request with invalid link name in the auxiliary point
 * 
 * @note the request must have a joint goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, invalidAuxLinkName)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircJointInterimCart("circ3_interim")};

  // empty path constraint
  planning_interface::MotionPlanRequest req = circ.toRequest();

  // empty path constraint
  req.path_constraints.position_constraints.front().link_name = "INVALID";
  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(req,res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME) << req;
}

/**
 * @brief test the circ planner with wrong center point
 */
TEST_P(TrajectoryGeneratorCIRCTest, wrongCenter)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.y += 1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(),res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with colinear start/goal/center position
 */
TEST_P(TrajectoryGeneratorCIRCTest, colinearCenter)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  // Stretch auxiliary and goal pose along line
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 0.05;
  circ.getGoalConfiguration().getPose().position.x += 0.1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(),res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN);
}

/**
 * @brief test the circ planner with  colinear start/goal/interim position
 */
TEST_P(TrajectoryGeneratorCIRCTest, colinearInterim)
{
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartInterimCart("circ3_interim")};
  circ.getAuxiliaryConfiguration().getConfiguration().setPose(circ.getStartConfiguration().getPose());
  circ.getGoalConfiguration().setPose(circ.getStartConfiguration().getPose());

  // Stretch auxiliary and goal pose along line
  circ.getAuxiliaryConfiguration().getConfiguration().getPose().position.x += 0.05;
  circ.getGoalConfiguration().getPose().position.x += 0.1;

  planning_interface::MotionPlanResponse res;
  EXPECT_FALSE(circ_->generate(circ.toRequest(),res));
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
  // Define auxiliary point and goal to be the same as the start
  auto circ {tdp_->getCircCartInterimCart("circ3_interim")};

  planning_interface::MotionPlanResponse res;
  ASSERT_TRUE(circ_->generate(circ.toRequest(),res));
  EXPECT_EQ(res.error_code_.val, moveit_msgs::MoveItErrorCodes::SUCCESS);
}

/**
 * @brief test the circ planner with center point and joint goal
 */
TEST_P(TrajectoryGeneratorCIRCTest, centerPointJointGoal)
{
  SKIP_IF_GRIPPER

  auto circ {tdp_->getCircJointCenterCart("circ1_center_2")};
  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  // construct planning request
  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircJointCenterCart("circ1_center_2")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();
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
  auto circ {tdp_->getCircCartCenterCart("circ1_center_2")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  SKIP_IF_GRIPPER

  auto circ {tdp_->getCircJointInterimCart("circ3_interim")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  SKIP_IF_GRIPPER

  auto circ {tdp_->getCircJointInterimCart("circ3_interim")};

  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
  auto circ {tdp_->getCircJointInterimCart("circ3_interim")};
  moveit_msgs::MotionPlanRequest req = circ.toRequest();

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
