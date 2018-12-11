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

#ifndef MOTION_PLAN_REQUEST_DIRECTOR_H
#define MOTION_PLAN_REQUEST_DIRECTOR_H

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include "pilz_industrial_motion_testutils/testdata_loader.h"

namespace pilz_industrial_motion_testutils {

/**
 * @brief Takes order from user and directs the builder to build the motion plan request
 * according to the order
 */
class MotionPlanRequestDirector
{
public:
  MotionPlanRequestDirector(){}

  /**
   * @brief Construct a LIN motion plan request with goal constraint given in joint space.
   * @param robot_model
   * @param motion_cmd
   * @return
   */
  moveit_msgs::MotionPlanRequest getLINJointReq(const robot_model::RobotModelConstPtr& robot_model,
                                                const STestMotionCommand& cmd);

  /**
   * @brief Construct a LIN motion plan request with goal constraint given in Cartesian space.
   * @param robot_model
   * @param motion_cmd
   * @return
   */
  moveit_msgs::MotionPlanRequest getLINCartReq(const robot_model::RobotModelConstPtr& robot_model,
                                               const STestMotionCommand& cmd);


  /**
   * @brief Construct a CIRC motion plan request with goal constraint given in joint space.
   * @param robot_model
   * @param motion_cmd: auxiliary pose types must be set as input
   * @return
   */
  moveit_msgs::MotionPlanRequest getCIRCJointReq(const robot_model::RobotModelConstPtr& robot_model,
                                                 const STestMotionCommand& cmd);

  /**
   * @brief Construct a CIRC motion plan request with goal constraint given in Cartesian space.
   * @param robot_model
   * @param motion_cmd: auxiliary pose types must be set as input
   * @return
   */
  moveit_msgs::MotionPlanRequest getCIRCCartReq(const robot_model::RobotModelConstPtr& robot_model,
                                                const STestMotionCommand& cmd);

private:
  /**
   * @brief Transform a vector of position and quanterion (x, y, z, wx, wy, wz, w) to Eigen::Affine3d.
   * @param pose
   * @return
   */
  Eigen::Isometry3d rawQuatVectorToEigen(const std::vector<double> &pose);

  /**
   * @brief Construct a robot state from the start joint values in cmd.
   * @param robot_model
   * @param cmd
   * @return
   */
  robot_state::RobotState getStartStateFromJoints(const robot_model::RobotModelConstPtr& robot_model,
                                                  const STestMotionCommand& cmd);

  /**
   * @brief Construct a robot state from the start pose in cmd. The start joint position is used as ik seed.
   * @param robot_model
   * @param cmd
   * @return
   */
  robot_state::RobotState getStartStateFromPose(const robot_model::RobotModelConstPtr& robot_model,
                                                const STestMotionCommand& cmd);
  /**
   * @brief Construct a robot state from the goal joint values in cmd.
   * @param robot_model
   * @param cmd
   * @return
   */
  robot_state::RobotState getGoalStateFromJoints(const robot_model::RobotModelConstPtr& robot_model,
                                                 const STestMotionCommand& cmd);

  /**
   * @brief Construct a robot state from the goal pose in cmd. The goal joint position is used as ik seed.
   * @param robot_model
   * @param cmd
   * @return
   */
  robot_state::RobotState getGoalStateFromPose(const robot_model::RobotModelConstPtr& robot_model,
                                               const STestMotionCommand& cmd);

};

}

#endif // MOTION_PLAN_REQUEST_DIRECTOR_H
