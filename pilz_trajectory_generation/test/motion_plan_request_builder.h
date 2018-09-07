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

#ifndef MOTION_PLAN_REQUEST_BUILDER_H
#define MOTION_PLAN_REQUEST_BUILDER_H

#include <string>

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/robot_state/robot_state.h>

class MotionPlanRequestBuilder
{
public:
  MotionPlanRequestBuilder(const moveit_msgs::MotionPlanRequest& req);

  MotionPlanRequestBuilder(const std::string& group_name);

  moveit_msgs::MotionPlanRequest createPtp();
  moveit_msgs::MotionPlanRequest createLin();

  void setJointStartState(const sensor_msgs::JointState& start_joint_state);
  void setJointStartState(const robot_state::RobotState& start_state);

  void clearJointStartState();

  void setGoal(const std::string& goal_link_name, const geometry_msgs::PoseStamped goal_pose);

private:
  moveit_msgs::MotionPlanRequest req_templ_;
};

#endif // MOTION_PLAN_REQUEST_BUILDER_H


