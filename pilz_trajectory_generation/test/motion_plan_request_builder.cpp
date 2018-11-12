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

#include "motion_plan_request_builder.h"

#include <moveit/kinematic_constraints/utils.h>

#include "pilz_msgs/MotionSequenceRequest.h"

MotionPlanRequestBuilder::MotionPlanRequestBuilder(const moveit_msgs::MotionPlanRequest& req)
{
  req_templ_ = req;
}

MotionPlanRequestBuilder::MotionPlanRequestBuilder(const std::string& group_name)
{
  // Default values
  req_templ_.group_name = group_name;

  req_templ_.num_planning_attempts = 1;
  req_templ_.allowed_planning_time = 0.5;
  req_templ_.max_velocity_scaling_factor = 0.1;
  req_templ_.max_acceleration_scaling_factor = 0.1;
}

moveit_msgs::MotionPlanRequest MotionPlanRequestBuilder::createPtp()
{
  req_templ_.planner_id = "PTP";

  return req_templ_;
}

moveit_msgs::MotionPlanRequest MotionPlanRequestBuilder::createLin()
{
  req_templ_.planner_id = "LIN";

  return req_templ_;
}

void MotionPlanRequestBuilder::setJointStartState(const sensor_msgs::JointState& start_joint_state)
{
  req_templ_.start_state.joint_state = start_joint_state;
}

void MotionPlanRequestBuilder::setJointStartState(const robot_state::RobotState& start_state)
{
  req_templ_.start_state.joint_state.name = start_state.getVariableNames();
  req_templ_.start_state.joint_state.position = std::vector<double>(
        start_state.getVariablePositions(),
        start_state.getVariablePositions()+start_state.getVariableCount());
  req_templ_.start_state.joint_state.velocity = std::vector<double>(
        start_state.getVariableVelocities(),
        start_state.getVariableVelocities()+start_state.getVariableCount());
}

void MotionPlanRequestBuilder::clearJointStartState()
{
  req_templ_.start_state.joint_state.position.clear();
  req_templ_.start_state.joint_state.velocity.clear();
  req_templ_.start_state.joint_state.effort.clear();
  req_templ_.start_state.joint_state.name.clear();
}

void MotionPlanRequestBuilder::setGoal(const std::string& goal_link_name, const geometry_msgs::PoseStamped goal_pose)
{
  req_templ_.goal_constraints.clear();
  req_templ_.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(goal_link_name, goal_pose));
}
