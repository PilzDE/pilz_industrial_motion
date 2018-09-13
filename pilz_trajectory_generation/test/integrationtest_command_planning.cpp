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

#include <iostream>
#include <string>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <pilz_industrial_motion_testutils/xml_testdata_loader.h>
#include <pilz_industrial_motion_testutils/motion_plan_request_director.h>

#include "test_utils.h"

const double EPSILON=1.0e-6;
const std::string PLAN_SERVICE_NAME = "/plan_kinematic_path";

// Parameters from parameter server
const std::string PARAM_PLANNING_GROUP_NAME("planning_group");
const std::string POSE_TRANSFORM_MATRIX_NORM_TOLERANCE("pose_norm_tolerance");
const std::string ROTATION_AXIS_NORM_TOLERANCE("rot_axis_norm_tolerance");
const std::string PARAM_TARGET_LINK_NAME("target_link");
const std::string TEST_DATA_FILE_NAME("testdata_file_name");
const std::string JOINT_PREFIX("joint_prefix");

class IntegrationTestCommandPlanning : public ::testing::Test
{
protected:
  virtual void SetUp();

protected:
  robot_model::RobotModelPtr robot_model_;

  double pose_norm_tolerance_, rot_axis_norm_tolerance_;
  std::string planning_group_, target_link_, test_data_file_name_;

  std::unique_ptr<pilz_industrial_motion_testutils::TestdataLoader> test_data_;

  std::string joint_prefix_ {testutils::JOINT_NAME_PREFIX};

  unsigned int num_joints_ {0};
};

void IntegrationTestCommandPlanning::SetUp()
{
  // create robot model
  robot_model_loader::RobotModelLoader model_loader;
  robot_model_ = model_loader.getModel();

  ros::NodeHandle ph("~");

  // get the parameters
  ASSERT_TRUE(ph.getParam(PARAM_PLANNING_GROUP_NAME, planning_group_));
  ASSERT_TRUE(ph.getParam(POSE_TRANSFORM_MATRIX_NORM_TOLERANCE, pose_norm_tolerance_));
  ASSERT_TRUE(ph.getParam(PARAM_TARGET_LINK_NAME, target_link_));
  ASSERT_TRUE(ph.getParam(ROTATION_AXIS_NORM_TOLERANCE, rot_axis_norm_tolerance_));
  ASSERT_TRUE(ph.getParam(TEST_DATA_FILE_NAME, test_data_file_name_));

  ph.getParam(JOINT_PREFIX, joint_prefix_);

  // check robot model
  testutils::checkRobotModel(robot_model_, planning_group_, target_link_);

  // load the test data provider
  test_data_.reset(new pilz_industrial_motion_testutils::XmlTestdataLoader{test_data_file_name_});
  ASSERT_NE(nullptr, test_data_) << "Failed to load test data by provider.";

  num_joints_ = robot_model_->getJointModelGroup(planning_group_)->getActiveJointModelNames().size();
}

/**
 * @brief Integration test for the motion planning of motion commands
 * Sends a planning request. Checks if response is obtained and if the response corresponds to the
 * motion request.
 *
 *  - Test Sequence:
 *    1. Generate request with joint goal and start state call planning service.
 *
 *  - Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 */
TEST_F(IntegrationTestCommandPlanning, PTPJointGoal)
{
  ros::NodeHandle node_handle("~");

  planning_interface::MotionPlanRequest req;

  std::vector<double> joints;

  // Generate start state
  ASSERT_TRUE(test_data_->getJoints("ZeroPose",  planning_group_, joints));
  sensor_msgs::JointState start_state = testutils::generateJointState(joints, joint_prefix_);
  // The goal
  ASSERT_TRUE(test_data_->getJoints("PTPJointValid",  planning_group_, joints));
  moveit_msgs::Constraints gc = testutils::generateJointConstraint(joints, joint_prefix_);
  req.goal_constraints.push_back(gc);

  // Scaling factor
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;
  srv.request.motion_plan_request.start_state.joint_state = start_state;

  // select planner by parameter
  srv.request.motion_plan_request.group_name = planning_group_;
  srv.request.motion_plan_request.planner_id = "PTP";

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(10));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory = response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_) << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0) << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for(trajectory_msgs::JointTrajectoryPoint point : trajectory.points)
  {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  for(size_t i = 0; i < num_joints_; ++i)
  {
    EXPECT_NEAR(trajectory.points.back().positions.at(i), gc.joint_constraints.at(i).position, 10e-10);
    EXPECT_NEAR(trajectory.points.back().velocities.at(i), 0, 10e-10);
    // EXPECT_NEAR(trajectory.points.back().accelerations.at(i), 0, 10e-10); // TODO what is expected
  }

}

/**
 * @brief Integration test for the motion planning of motion commands
 * Sends a planning request. Checks if response is obtained and if the response corresponds to the
 * motion request.
 *
 *  - Test Sequence:
 *    1. Generate request with pose goal and start state call planning service.
 *
 *  - Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 */
TEST_F(IntegrationTestCommandPlanning, PTPPoseGoal)
{
  ros::NodeHandle node_handle("~");

  planning_interface::MotionPlanRequest req;

  // The start state
  std::vector<double> joints;
  ASSERT_TRUE(test_data_->getJoints("ZeroPose",  planning_group_, joints));
  sensor_msgs::JointState start_state = testutils::generateJointState(joints, joint_prefix_);

  // Generate start state
  std::vector<double> pose_vec;
  ASSERT_TRUE(test_data_->getPose("PTPPose",  planning_group_, pose_vec));

  // The goal
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "";
  pose.pose = pilz_industrial_motion_testutils::TestdataLoader::fromVecToMsg(pose_vec);
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(target_link_,
                                                      pose,
                                                      tolerance_pose,
                                                      tolerance_angle);
  req.goal_constraints.push_back(pose_goal);
  req.group_name = planning_group_;

  // Scaling factor
  req.max_velocity_scaling_factor = 1.0;
  req.max_acceleration_scaling_factor = 1.0;

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;
  srv.request.motion_plan_request.start_state.joint_state = start_state;

  // select planner by parameter
  srv.request.motion_plan_request.group_name = planning_group_;
  srv.request.motion_plan_request.planner_id = "PTP";

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(10));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory = response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_) << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0) << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for(trajectory_msgs::JointTrajectoryPoint point : trajectory.points)
  {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  // TODO check that at right position
  robot_state::RobotState rstate(robot_model_);
  rstate.setJointGroupPositions(planning_group_,response.trajectory.joint_trajectory.points.back().positions);
  rstate.update();
  Eigen::Affine3d tf = rstate.getFrameTransform(target_link_);

  geometry_msgs::Pose goal_pose;
  tf::poseEigenToMsg(tf, goal_pose);


  EXPECT_NEAR(tf(0,3), pose_vec[0], EPSILON);
  EXPECT_NEAR(tf(1,3), pose_vec[1], EPSILON);
  EXPECT_NEAR(tf(2,3), pose_vec[2], EPSILON);


  Eigen::Affine3d expec_pose;
  tf::poseMsgToEigen(pose.pose, expec_pose);

  EXPECT_TRUE(Eigen::Quaterniond(tf.rotation()).isApprox(Eigen::Quaterniond(expec_pose.rotation()), EPSILON));
}

/**
 * @brief Tests motion planning for linear motion commands
 * (Goal given as joint position).
 *
 * PLEASE NOTE: Parameter tests or more generally speaking detailed lin tests
 * are done via unit tests. With the help of the integration tests, it is
 * only checked that a linear command actually performs a linear command.
 *
 * Test Sequence:
 *  1. Generate request and make service request.
 *  2. Check if target position correct.
 *  3. Check if trajectory is linear.
 *
 * Expected Results:
 *  1. Planning request is successful.
 *  2. Goal position correponds with the given goal position.
 *  3. Trajectory is a straight line.
 */
TEST_F(IntegrationTestCommandPlanning, LinJointGoal)
{
  ros::NodeHandle node_handle("~");

  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  lin_cmd.planning_group=planning_group_;
  ASSERT_TRUE(test_data_->getLin("LINCmd1", lin_cmd));

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 1 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  // The start state
  std::vector<double> joints;
  ASSERT_TRUE(test_data_->getJoints("ZeroPose",  planning_group_, joints));
  sensor_msgs::JointState start_state = testutils::generateJointState(lin_cmd.start_position, joint_prefix_);

  // The goal
  moveit_msgs::Constraints gc = testutils::generateJointConstraint(lin_cmd.goal_position, joint_prefix_);

  planning_interface::MotionPlanRequest req;
  req.group_name = planning_group_;
  req.planner_id = "LIN";
  req.start_state.joint_state = start_state;
  req.goal_constraints.push_back(gc);
  req.max_velocity_scaling_factor = lin_cmd.vel_scale;
  req.max_acceleration_scaling_factor = lin_cmd.acc_scale;

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(testutils::DEFAULT_SERVICE_TIMEOUT));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 2 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  ASSERT_TRUE(testutils::isGoalReached(robot_model_,
                                       response.trajectory.joint_trajectory,
                                       req,
                                       pose_norm_tolerance_)) << "Goal not reached.";

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 3 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  ASSERT_TRUE(testutils::checkCartesianLinearity(
                robot_model_,
                response.trajectory.joint_trajectory,
                req,
                pose_norm_tolerance_,
                rot_axis_norm_tolerance_)) << "Trajectory violates cartesian linearity.";
}

/**
 * @brief Tests motion planning for linear motion commands
 * (Goal given as cartesian position).
 *
 * PLEASE NOTE: Parameter tests or more generally speaking detailed lin tests
 * are done via unit tests. With the help of the integration tests,
 * it is only checked that a linear command actually performs a linear command.
 *
 * Test Sequence:
 *  1. Generate request and make service request.
 *  2. Check if target position correct.
 *  3. Check if trajectory is linear.
 *
 * Expected Results:
 *  1. Planning request is successful.
 *  2. Goal position correponds with the given goal position.
 *  3. Trajectory is a straight line.
 */
TEST_F(IntegrationTestCommandPlanning, LinPosGoal)
{
  ros::NodeHandle node_handle("~");

  pilz_industrial_motion_testutils::STestMotionCommand lin_cmd;
  lin_cmd.planning_group=planning_group_;
  ASSERT_TRUE(test_data_->getLin("LINCmd1", lin_cmd));

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 1 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  // The start state
  std::vector<double> joints;
  ASSERT_TRUE(test_data_->getJoints("ZeroPose",  planning_group_, joints));
  sensor_msgs::JointState start_state = testutils::generateJointState(lin_cmd.start_position, joint_prefix_);

  // The goal
  geometry_msgs::PoseStamped stampedGoalPose;
  stampedGoalPose.header.frame_id = "";
  testutils::toTCPPose( robot_model_, target_link_, lin_cmd.goal_position,
                        stampedGoalPose.pose, joint_prefix_);

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(target_link_,
                                                      stampedGoalPose);

  planning_interface::MotionPlanRequest req;
  req.group_name = planning_group_;
  req.planner_id = "LIN";
  req.start_state.joint_state = start_state;
  req.goal_constraints.push_back(pose_goal);
  req.max_velocity_scaling_factor = lin_cmd.vel_scale;
  req.max_acceleration_scaling_factor = lin_cmd.acc_scale;

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(testutils::DEFAULT_SERVICE_TIMEOUT));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 2 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  ASSERT_TRUE(testutils::isGoalReached(robot_model_,
                                       response.trajectory.joint_trajectory,
                                       req,
                                       pose_norm_tolerance_)) << "Goal not reached.";

  std::cout << "++++++++++" << std::endl;
  std::cout << "+ Step 3 +" << std::endl;
  std::cout << "++++++++++" << std::endl;

  ASSERT_TRUE(testutils::checkCartesianLinearity(
                robot_model_,
                response.trajectory.joint_trajectory,
                req,
                pose_norm_tolerance_,
                rot_axis_norm_tolerance_)) << "Trajectory violates cartesian linearity.";

}


/**
 * @brief Integration test for the motion planning of motion commands
 * Sends a planning request. Checks if response is obtained and if the response corresponds to the
 * motion request.
 *
 *  - Test Sequence:
 *    1. Generate request with JOINT goal and start state call planning service.
 *
 *  - Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 *    2. Waypoints are on the desired circle
 */
TEST_F(IntegrationTestCommandPlanning, CIRCJointGoal)
{
  ros::NodeHandle node_handle("~");

  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(test_data_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";

  pilz_industrial_motion_testutils::MotionPlanRequestDirector director;
  moveit_msgs::MotionPlanRequest req = director.getCIRCJointReq(robot_model_, circ_cmd);

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(10));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory = response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_) << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0) << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for(trajectory_msgs::JointTrajectoryPoint point : trajectory.points)
  {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  // check goal is reached
  ASSERT_TRUE(testutils::isGoalReached(robot_model_,
                                       response.trajectory.joint_trajectory,
                                       req,
                                       pose_norm_tolerance_)) << "Goal not reached.";

  // check all waypoints are on the circle and SLERP
  robot_state::RobotState waypoint_state(robot_model_);
  Eigen::Affine3d waypoint_pose;
  double x_dist, y_dist, z_dist;

  x_dist = circ_cmd.aux_pose[0] - circ_cmd.start_pose[0];
  y_dist = circ_cmd.aux_pose[1] - circ_cmd.start_pose[1];
  z_dist = circ_cmd.aux_pose[2] - circ_cmd.start_pose[2];
  double expected_radius = sqrt( x_dist*x_dist + y_dist*y_dist +  z_dist*z_dist );
  for(const auto & waypoint : trajectory.points)
  {
    waypoint_state.setJointGroupPositions(planning_group_, waypoint.positions);
    waypoint_pose = waypoint_state.getFrameTransform(target_link_);

    // Calculate (and check) distance of current trajectory waypoint from circ center
    x_dist = circ_cmd.aux_pose[0] - waypoint_pose(0,3);
    y_dist = circ_cmd.aux_pose[1] - waypoint_pose(1,3);
    z_dist = circ_cmd.aux_pose[2] - waypoint_pose(2,3);
    double actual_radius = sqrt( x_dist*x_dist + y_dist*y_dist +  z_dist*z_dist );
    EXPECT_NEAR(actual_radius, expected_radius, pose_norm_tolerance_) << "Trajectory way point is not on the circle.";

    // Check orientation
    Eigen::Affine3d start_pose, goal_pose;
    tf::poseMsgToEigen(pilz_industrial_motion_testutils::TestdataLoader::fromVecToMsg(circ_cmd.start_pose), start_pose);
    tf::poseMsgToEigen(pilz_industrial_motion_testutils::TestdataLoader::fromVecToMsg(circ_cmd.goal_pose), goal_pose);
    EXPECT_TRUE( testutils::checkSLERP(start_pose, goal_pose, waypoint_pose, rot_axis_norm_tolerance_) );
  }

}

/**
 * @brief Integration test for the motion planning of motion commands
 * Sends a planning request. Checks if response is obtained and if the response corresponds to the
 * motion request.
 *
 *  - Test Sequence:
 *    1. Generate request with POSE goal and start state call planning service.
 *
 *  - Expected Results:
 *    1. Last point of the resulting trajectory is at the goal
 *    2. Waypoints are on the desired circle
 */
TEST_F(IntegrationTestCommandPlanning, CIRCPoseGoal)
{
  ros::NodeHandle node_handle("~");

  pilz_industrial_motion_testutils::STestMotionCommand circ_cmd;
  circ_cmd.aux_pos_type = pilz_industrial_motion_testutils::ECircAuxPosType::eCENTER;
  ASSERT_TRUE(test_data_->getCirc("ValidCIRCCmd2", circ_cmd)) << "failed to get circ command from test data";

  pilz_industrial_motion_testutils::MotionPlanRequestDirector director;
  moveit_msgs::MotionPlanRequest req = director.getCIRCCartReq(robot_model_, circ_cmd);

  // Generate the service request
  moveit_msgs::GetMotionPlan srv;
  srv.request.motion_plan_request = req;

  ros::service::waitForService(PLAN_SERVICE_NAME, ros::Duration(10));
  ros::ServiceClient client = node_handle.serviceClient<moveit_msgs::GetMotionPlan>(PLAN_SERVICE_NAME);

  // Call the service client
  client.call(srv);

  // Obtain the response
  const moveit_msgs::MotionPlanResponse& response = srv.response.motion_plan_response;

  // Make sure the planning succeeded
  ASSERT_EQ(moveit_msgs::MoveItErrorCodes::SUCCESS, response.error_code.val) << "Planning failed!";

  // Check the result
  trajectory_msgs::JointTrajectory trajectory = response.trajectory.joint_trajectory;

  EXPECT_EQ(trajectory.joint_names.size(), num_joints_) << "Wrong number of jointnames";
  EXPECT_GT(trajectory.points.size(), 0) << "There are no points in the trajectory";

  // Check that every point has position, velocity, acceleration
  for(trajectory_msgs::JointTrajectoryPoint point : trajectory.points)
  {
    EXPECT_EQ(point.positions.size(), num_joints_);
    EXPECT_EQ(point.velocities.size(), num_joints_);
    EXPECT_EQ(point.accelerations.size(), num_joints_);
  }

  // check goal is reached
  ASSERT_TRUE(testutils::isGoalReached(robot_model_,
                                       response.trajectory.joint_trajectory,
                                       req,
                                       pose_norm_tolerance_)) << "Goal not reached.";

  // check all waypoints are on the cricle and SLERP
  robot_state::RobotState waypoint_state(robot_model_);
  Eigen::Affine3d waypoint_pose;
  double x_dist, y_dist, z_dist;

  x_dist = circ_cmd.aux_pose[0] - circ_cmd.start_pose[0];
  y_dist = circ_cmd.aux_pose[1] - circ_cmd.start_pose[1];
  z_dist = circ_cmd.aux_pose[2] - circ_cmd.start_pose[2];
  double expected_radius = sqrt( x_dist*x_dist + y_dist*y_dist +  z_dist*z_dist );
  for(const auto & waypoint : trajectory.points)
  {
    waypoint_state.setJointGroupPositions(planning_group_, waypoint.positions);
    waypoint_pose = waypoint_state.getFrameTransform(target_link_);

    // Calculate (and check) distance of current trajectory waypoint from circ center
    x_dist = circ_cmd.aux_pose[0] - waypoint_pose(0,3);
    y_dist = circ_cmd.aux_pose[1] - waypoint_pose(1,3);
    z_dist = circ_cmd.aux_pose[2] - waypoint_pose(2,3);
    double actual_radius = sqrt( x_dist*x_dist + y_dist*y_dist +  z_dist*z_dist );
    EXPECT_NEAR(actual_radius, expected_radius, pose_norm_tolerance_) << "Trajectory way point is not on the circle.";

    // Check orientation
    Eigen::Affine3d start_pose, goal_pose;
    tf::poseMsgToEigen(pilz_industrial_motion_testutils::TestdataLoader::fromVecToMsg(circ_cmd.start_pose), start_pose);
    tf::poseMsgToEigen(pilz_industrial_motion_testutils::TestdataLoader::fromVecToMsg(circ_cmd.goal_pose), goal_pose);
    EXPECT_TRUE(testutils::checkSLERP(start_pose,
                                      goal_pose,
                                      waypoint_pose,
                                      rot_axis_norm_tolerance_));
  }

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "integrationtest_command_planning");
  return RUN_ALL_TESTS();
}
