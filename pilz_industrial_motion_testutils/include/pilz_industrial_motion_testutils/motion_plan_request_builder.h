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

#ifndef MOTION_PLAN_REQUEST_BUILDER_H
#define MOTION_PLAN_REQUEST_BUILDER_H

#include <Eigen/Eigen>
#include <moveit_msgs/MotionPlanRequest.h>

namespace moveit { namespace core{ class RobotState; }}

namespace pilz_industrial_motion_testutils {

/**
 * @brief The MotionPlanRequestBuilder class builds up a complex motion plan request.
 * Part of builder pattern. Do not use this class directly. Use MotionPlanRequestDirector
 * class to construct specific requests.
 */
class MotionPlanRequestBuilder
{
public:
  MotionPlanRequestBuilder(){}

  moveit_msgs::MotionPlanRequest getRequest();

  void setPlannerID(const std::string& id);
  void setPlanningGroup(const std::string& group);
  void setStartState(const moveit::core::RobotState& start_state);
  void setScalingFactor(double velocity_scale, double acc_scale);
  void setGoalConstraint(const std::string& link, const Eigen::Affine3d& goal_pose);
  void setGoalConstraint(const std::string& link, const geometry_msgs::PoseStamped goal_pose);
  void setGoalConstraint(const std::string& group, const moveit::core::RobotState& goal_state);
  /**
   * @brief set the CIRC auxiliary point as a path constraint
   * @param constraint_name: center or interim
   * @param link_name
   * @param aux_state
   */
  void setCIRCAuxiliaryConstraint(const std::string& constraint_name, const std::string& link_name,
                            const moveit::core::RobotState &aux_state);
  /**
   * @brief set the CIRC auxiliary point as a path constraint
   * @param constraint_name: center or interim
   * @param link_name
   * @param x
   * @param y
   * @param z
   */
  void setCIRCAuxiliaryConstraint(const std::string& constraint_name, const std::string& link_name,
                            double x, double y, double z);
private:
  moveit_msgs::MotionPlanRequest req_msg_;
};

}

#endif // MOTION_PLAN_REQUEST_BUILDER_H


