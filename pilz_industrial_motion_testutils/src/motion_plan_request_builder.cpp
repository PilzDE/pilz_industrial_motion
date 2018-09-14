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

#include "pilz_industrial_motion_testutils/motion_plan_request_builder.h"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/PositionConstraint.h>

namespace pilz_industrial_motion_testutils {

moveit_msgs::MotionPlanRequest MotionPlanRequestBuilder::getRequest()
{
  return req_msg_;
}

void MotionPlanRequestBuilder::setPlannerID(const std::string &id)
{
  req_msg_.planner_id = id;
}

void MotionPlanRequestBuilder::setPlanningGroup(const std::string &group)
{
  req_msg_.group_name = group;
}

void MotionPlanRequestBuilder::setStartState(const robot_state::RobotState &start_state)
{
  moveit::core::robotStateToRobotStateMsg(start_state, req_msg_.start_state, true);
}

void MotionPlanRequestBuilder::setScalingFactor(double velocity_scale, double acc_scale)
{
  req_msg_.max_velocity_scaling_factor = velocity_scale;
  req_msg_.max_acceleration_scaling_factor = acc_scale;
}

void MotionPlanRequestBuilder::setGoalConstraint(const std::string &link, const Eigen::Affine3d &goal_pose)
{
  geometry_msgs::PoseStamped pose;
  tf::poseEigenToMsg(goal_pose, pose.pose);
  setGoalConstraint(link, pose);
}

void MotionPlanRequestBuilder::setGoalConstraint(const std::string &link, const geometry_msgs::PoseStamped goal_pose)
{
  req_msg_.goal_constraints.clear();
  req_msg_.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(link, goal_pose));
}

void MotionPlanRequestBuilder::setGoalConstraint(const std::string &group,
                                                 const moveit::core::RobotState &goal_state)
{
  req_msg_.goal_constraints.clear();
  req_msg_.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(goal_state,
                                                        goal_state.getRobotModel()->getJointModelGroup(group)));
}

void MotionPlanRequestBuilder::setCIRCAuxiliaryConstraint(const std::string &constraint_name, const std::string &link_name,
                                                    const moveit::core::RobotState &aux_state)
{
    req_msg_.path_constraints.name = constraint_name;
    req_msg_.path_constraints.position_constraints.clear();
    moveit_msgs::PositionConstraint p_constraint;
    p_constraint.link_name = link_name;
    Eigen::Affine3d aux_pose = aux_state.getFrameTransform(link_name);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(aux_pose, pose);
    p_constraint.constraint_region.primitive_poses.push_back(pose);
    req_msg_.path_constraints.position_constraints.clear();
    req_msg_.path_constraints.position_constraints.push_back(p_constraint);
}

void MotionPlanRequestBuilder::setCIRCAuxiliaryConstraint(const std::string &constraint_name,
                                                    const std::string &link_name, double x, double y, double z)
{
  req_msg_.path_constraints.name = constraint_name;
  req_msg_.path_constraints.position_constraints.clear();
  moveit_msgs::PositionConstraint p_constraint;
  p_constraint.link_name = link_name;
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  p_constraint.constraint_region.primitive_poses.push_back(pose);
  req_msg_.path_constraints.position_constraints.clear();
  req_msg_.path_constraints.position_constraints.push_back(p_constraint);
}

}
