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

#ifndef CHECKS_H
#define CHECKS_H

#include <gtest/gtest.h>
#include <moveit/robot_state/robot_state.h>

namespace pilz_industrial_motion_testutils
{

::testing::AssertionResult isAtExpectedPosition(const robot_state::RobotState& expected,
                                                const robot_state::RobotState& actual,
                                                const double epsilon)
{
  if(expected.getVariableCount() != actual.getVariableCount())
  {
    return ::testing::AssertionFailure() << "Both states have different number of Variables";
  }

  for(size_t i = 0; i < actual.getVariableCount(); ++i)
  {
    // PLEASE NOTE: This comparision only works for reasonably
    // values. That means: Values are not to large, values are
    // reasonably close by each other.
    if (std::fabs(expected.getVariablePosition(i) - actual.getVariablePosition(i)) > epsilon)
    {
      std::stringstream msg;
      msg << expected.getVariableNames().at(i) << " position - expected: "
          << expected.getVariablePosition(i) << " actual: " << actual.getVariablePosition(i);

      return ::testing::AssertionFailure() << msg.str();
    }
  }

  return ::testing::AssertionSuccess();
}

}

#endif // CENTERAUXILIARY_H