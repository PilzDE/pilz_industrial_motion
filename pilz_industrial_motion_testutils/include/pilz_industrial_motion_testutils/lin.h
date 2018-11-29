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

#ifndef LIN_H
#define LIN_H

#include <stdexcept>

#include "basecmd.h"

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Data class storing all information regarding a linear command.
 */
template <class StartType, class GoalType>
class Lin : public BaseCmd<StartType, GoalType>
{
public:
  Lin()
    : BaseCmd<StartType, GoalType>()
  {}

public:
  planning_interface::MotionPlanRequest toRequest() const override;

};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class GoalType>
inline planning_interface::MotionPlanRequest Lin<StartType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "LIN";
  req.group_name = this->planning_group_;

  req.max_velocity_scaling_factor = this->vel_scale_;
  req.max_acceleration_scaling_factor = this->acc_scale_;

  req.start_state = this->start_.toMoveitMsgsRobotState();
  req.goal_constraints.push_back(this->goal_.toGoalConstraints());

  return req;
}

}

#endif // LIN_H
