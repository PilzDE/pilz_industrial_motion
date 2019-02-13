/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef CARTESIANCONFIGURATION_H
#define CARTESIANCONFIGURATION_H

#include <stdexcept>
#include <vector>
#include <sstream>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

#include "robotconfiguration.h"

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Class to define a robot configuration in space
 * with the help of cartesian coordinates.
 */
class CartesianConfiguration : public RobotConfiguration
{
public:
  CartesianConfiguration();

  CartesianConfiguration(const std::string& group_name,
                         const std::string& link_name,
                         const std::vector<double>& config);

  CartesianConfiguration(const std::string& group_name,
                         const std::string& link_name,
                         const std::vector<double>& config,
                         moveit::core::RobotModelConstPtr robot_model);

public:
  virtual moveit_msgs::Constraints toGoalConstraints() const override;
  virtual moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

  const std::string& getLinkName() const;
  const geometry_msgs::Pose &getPose() const;

private:
  static geometry_msgs::Pose toPose(const std::vector<double>& pose);
  static geometry_msgs::PoseStamped toStampedPose(const geometry_msgs::Pose& pose);

private:
  std::string link_name_;
  geometry_msgs::Pose pose_;
};

inline const std::string& CartesianConfiguration::getLinkName() const
{
  return link_name_;
}

inline const geometry_msgs::Pose& CartesianConfiguration::getPose() const
{
  return pose_;
}

inline moveit_msgs::Constraints CartesianConfiguration::toGoalConstraints() const
{
  return kinematic_constraints::constructGoalConstraints(link_name_, toStampedPose(pose_));
}

}

#endif // CARTESIANCONFIGURATION_H
