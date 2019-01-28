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

#ifndef CIRC_H
#define CIRC_H

#include <memory>

#include "basecmd.h"
#include "circauxiliary.h"

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Data class storing all information regarding a Circ command.
 */
template <class StartType, class AuxiliaryType, class GoalType>
class Circ : public BaseCmd<StartType, GoalType>
{
public:
  Circ()
    : BaseCmd<StartType, GoalType>()
  {}

public:
  void setAuxiliaryConfiguration(AuxiliaryType auxiliary);

public:
  planning_interface::MotionPlanRequest toRequest() const override;

private:
  AuxiliaryType auxiliary_;

};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class AuxiliaryType, class GoalType>
inline void Circ<StartType, AuxiliaryType, GoalType>::setAuxiliaryConfiguration(AuxiliaryType auxiliary)
{
  auxiliary_ = auxiliary;
}

template <class StartType, class AuxiliaryType, class GoalType>
inline planning_interface::MotionPlanRequest Circ<StartType, AuxiliaryType, GoalType>::toRequest() const
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "CIRC";
  req.group_name = this->planning_group_;

  req.max_velocity_scaling_factor = this->vel_scale_;
  req.max_acceleration_scaling_factor = this->acc_scale_;

  req.start_state = this->start_.toMoveitMsgsRobotState();
  req.path_constraints = auxiliary_.toPathConstraints();
  req.goal_constraints.push_back(this->goal_.toGoalConstraints());

  return req;
}

}

#endif // CIRC_H
