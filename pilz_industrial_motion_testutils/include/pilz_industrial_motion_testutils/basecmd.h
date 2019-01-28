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

#ifndef BASECMD_H
#define BASECMD_H

#include <string>

#include "motioncmd.h"

namespace pilz_industrial_motion_testutils
{

/**
 * @brief Base class for all command data classes.
 */
template <class StartType, class GoalType>
class BaseCmd : public MotionCmd
{
public:
  BaseCmd()
    : MotionCmd()
  {}

public:

  void setStartConfiguration(StartType start);
  void setGoalConfiguration(GoalType goal);

  StartType& getStartConfiguration();
  GoalType& getGoalConfiguration();

protected:
  GoalType goal_;
  StartType start_;

};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setStartConfiguration(StartType start)
{
  start_ = start;
}

template <class StartType, class GoalType>
inline void BaseCmd<StartType, GoalType>::setGoalConfiguration(GoalType goal)
{
  goal_ = goal;
}

template <class StartType, class GoalType>
inline StartType& BaseCmd<StartType, GoalType>::getStartConfiguration()
{
  return start_;
}

template <class StartType, class GoalType>
inline GoalType& BaseCmd<StartType, GoalType>::getGoalConfiguration()
{
  return goal_;
}

}

#endif // BASECMD_H
