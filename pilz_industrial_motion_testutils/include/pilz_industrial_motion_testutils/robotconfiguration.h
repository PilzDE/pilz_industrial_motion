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

#ifndef ROBOTCONFIGURATION_H
#define ROBOTCONFIGURATION_H

#include <string>

#include <moveit/robot_model/robot_model.h>

#include "goalconstraintsmsgconvertible.h"
#include "robotstatemsgconvertible.h"

namespace pilz_industrial_motion_testutils
{

class RobotConfiguration : public RobotStateMsgConvertible, public GoalConstraintMsgConvertible
{
public:
  RobotConfiguration();

  RobotConfiguration(const std::string& group_name);

  RobotConfiguration(const std::string& group_name,
                     moveit::core::RobotModelConstPtr robot_model);

public:
  void setRobotModel(moveit::core::RobotModelConstPtr robot_model);
  void setGroupName(const std::string& group_name);

  virtual moveit_msgs::Constraints toGoalConstraints() const override;
  virtual moveit_msgs::RobotState toMoveitMsgsRobotState() const override;

protected:
  std::string group_name_;
  moveit::core::RobotModelConstPtr robot_model_;

};

inline void RobotConfiguration::setRobotModel(moveit::core::RobotModelConstPtr robot_model)
{
  robot_model_ = robot_model;
}

inline void RobotConfiguration::setGroupName(const std::string& group_name)
{
  group_name_ = group_name;
}

inline moveit_msgs::Constraints RobotConfiguration::toGoalConstraints() const
{
  throw std::runtime_error("Function 'toConstraints()' not implemented");
}

inline moveit_msgs::RobotState RobotConfiguration::toMoveitMsgsRobotState() const
{
  throw std::runtime_error("Function 'toMoveitMsgsRobotState()' not implemented");
}

}

#endif // ROBOTCONFIGURATION_H
