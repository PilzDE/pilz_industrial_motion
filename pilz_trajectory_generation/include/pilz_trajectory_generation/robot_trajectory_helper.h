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

#ifndef ROBOT_TRAJECTORY_HELPER_H
#define ROBOT_TRAJECTORY_HELPER_H

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace pilz_trajectory_generation
{

class RobotTrajectoryHelper
{
public:
    static void appendWithoutRedundantFirstState(robot_trajectory::RobotTrajectoryPtr &robot_trajectory,
                                                 robot_trajectory::RobotTrajectory &tail);

    // Constant to check for equality of values.
    static constexpr double ROBOT_STATE_EQUALITY_EPSILON = 1e-4;
};

}  // namespace pilz_trajectory_generation

#endif  // ROBOT_TRAJECTORY_HELPER_H
