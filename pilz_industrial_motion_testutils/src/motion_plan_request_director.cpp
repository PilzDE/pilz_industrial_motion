/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pilz_industrial_motion_testutils/motion_plan_request_director.h"
#include "pilz_industrial_motion_testutils/motion_plan_request_builder.h"

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>

namespace pilz_industrial_motion_testutils {

moveit_msgs::MotionPlanRequest MotionPlanRequestDirector::getLINJointReq(const moveit::core::RobotModelConstPtr &robot_model,
                                                                         const STestMotionCommand &cmd)
{
  MotionPlanRequestBuilder builder;
  builder.setPlannerID("LIN");
  builder.setPlanningGroup(cmd.planning_group);

  // set start state
  builder.setStartState(getStartStateFromJoints(robot_model, cmd));

  // set goal constraint in joint space
  builder.setGoalConstraint(cmd.planning_group, getGoalStateFromJoints(robot_model, cmd));

  // set scaling factor
  builder.setScalingFactor(cmd.vel_scale, cmd.acc_scale);

  return builder.getRequest();
}

moveit_msgs::MotionPlanRequest MotionPlanRequestDirector::getLINCartReq(const moveit::core::RobotModelConstPtr &robot_model,
                                                                        const STestMotionCommand &cmd)
{
  MotionPlanRequestBuilder builder;
  builder.setPlannerID("LIN");
  builder.setPlanningGroup(cmd.planning_group);

  // set start state
  builder.setStartState(getStartStateFromJoints(robot_model, cmd));

  // set goal constraint in Cartesian space
  builder.setGoalConstraint(cmd.target_link, getGoalStateFromJoints(robot_model, cmd).getFrameTransform(cmd.target_link));

  // set scaling factor
  builder.setScalingFactor(cmd.vel_scale, cmd.acc_scale);

  return builder.getRequest();
}

moveit_msgs::MotionPlanRequest MotionPlanRequestDirector::getCIRCJointReq(const moveit::core::RobotModelConstPtr &robot_model,
                                                                          const STestMotionCommand &cmd)
{
  MotionPlanRequestBuilder builder;
  builder.setPlannerID("CIRC");
  builder.setPlanningGroup(cmd.planning_group);

  // set start state
  builder.setStartState(getStartStateFromPose(robot_model, cmd));

  // set goal constraint in joint space
  builder.setGoalConstraint(cmd.planning_group, getGoalStateFromPose(robot_model, cmd));

  // set aux point as path constraint
  switch(cmd.aux_pos_type)
  {
  case ECircAuxPosType::eCENTER:
    builder.setCIRCAuxiliaryConstraint("center", cmd.target_link, cmd.aux_pose.at(0),
                                       cmd.aux_pose.at(1), cmd.aux_pose.at(2));
    break;
  case ECircAuxPosType::eINTERMEDIATE:
    builder.setCIRCAuxiliaryConstraint("interim", cmd.target_link, cmd.aux_pose.at(0),
                                       cmd.aux_pose.at(1), cmd.aux_pose.at(2));
    break;
  }

  // set scaling factor
  builder.setScalingFactor(cmd.vel_scale, cmd.acc_scale);

  return builder.getRequest();
}

moveit_msgs::MotionPlanRequest MotionPlanRequestDirector::getCIRCCartReq(const moveit::core::RobotModelConstPtr &robot_model,
                                                                         const STestMotionCommand &cmd)
{
  MotionPlanRequestBuilder builder;
  builder.setPlannerID("CIRC");
  builder.setPlanningGroup(cmd.planning_group);

  // set start state
  builder.setStartState(getStartStateFromPose(robot_model, cmd));

  // set goal constraint in Cartesian space
  builder.setGoalConstraint(cmd.target_link, rawQuatVectorToEigen(cmd.goal_pose));

  // set aux point as path constraint
  switch(cmd.aux_pos_type)
  {
  case ECircAuxPosType::eCENTER:
    builder.setCIRCAuxiliaryConstraint("center", cmd.target_link, cmd.aux_pose.at(0),
                                       cmd.aux_pose.at(1), cmd.aux_pose.at(2));
    break;
  case ECircAuxPosType::eINTERMEDIATE:
    builder.setCIRCAuxiliaryConstraint("interim", cmd.target_link, cmd.aux_pose.at(0),
                                       cmd.aux_pose.at(1), cmd.aux_pose.at(2));
    break;
  }

  // set scaling factor
  builder.setScalingFactor(cmd.vel_scale, cmd.acc_scale);

  return builder.getRequest();
}

Eigen::Isometry3d MotionPlanRequestDirector::rawQuatVectorToEigen(const std::vector<double>& pose)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = pose.at(0);
  pose_msg.position.y = pose.at(1);
  pose_msg.position.z = pose.at(2);
  pose_msg.orientation.x = pose.at(3);
  pose_msg.orientation.y = pose.at(4);
  pose_msg.orientation.z = pose.at(5);
  pose_msg.orientation.w = pose.at(6);

  Eigen::Isometry3d pose_eigen;
  tf::poseMsgToEigen(pose_msg, pose_eigen);
  return pose_eigen;
}

moveit::core::RobotState MotionPlanRequestDirector::getStartStateFromJoints(
    const moveit::core::RobotModelConstPtr &robot_model, const STestMotionCommand &cmd)
{
  robot_state::RobotState rstate(robot_model);
  rstate.setToDefaultValues();
  rstate.setJointGroupPositions(cmd.planning_group, cmd.start_position);
  rstate.update();

  return rstate;
}

moveit::core::RobotState MotionPlanRequestDirector::getStartStateFromPose(
    const moveit::core::RobotModelConstPtr &robot_model, const STestMotionCommand &cmd)
{
  // set the joints as ik seed
  robot_state::RobotState rstate = getStartStateFromJoints(robot_model, cmd);

  // set to Cartesian pose
  Eigen::Isometry3d start_pose = rawQuatVectorToEigen(cmd.start_pose);
  if(!rstate.setFromIK(rstate.getRobotModel()->getJointModelGroup(cmd.planning_group), start_pose, cmd.target_link))
  {
    ROS_ERROR_STREAM("no solution for ik \n" << start_pose.translation() << "\n" << start_pose.linear());
    throw std::runtime_error("no solution for inverse kinematics");
  }
  return rstate;
}

moveit::core::RobotState MotionPlanRequestDirector::getGoalStateFromJoints(
    const moveit::core::RobotModelConstPtr &robot_model, const STestMotionCommand &cmd)
{
  robot_state::RobotState rstate(robot_model);
  rstate.setToDefaultValues();
  rstate.setJointGroupPositions(cmd.planning_group, cmd.goal_position);
  rstate.update();

  return rstate;
}

moveit::core::RobotState MotionPlanRequestDirector::getGoalStateFromPose(
    const moveit::core::RobotModelConstPtr &robot_model, const STestMotionCommand &cmd)
{
  // set the joints as ik seed
  robot_state::RobotState rstate = getGoalStateFromJoints(robot_model, cmd);

  // set to Cartesian pose
  Eigen::Isometry3d goal_pose = rawQuatVectorToEigen(cmd.goal_pose);
  if(!rstate.setFromIK(rstate.getRobotModel()->getJointModelGroup(cmd.planning_group), goal_pose, cmd.target_link))
  {
    ROS_ERROR_STREAM("no solution for ik \n" << goal_pose.translation() << "\n" << goal_pose.linear());
    throw std::runtime_error("no solution for inverse kinematics");
  }
  return rstate;
}

}
