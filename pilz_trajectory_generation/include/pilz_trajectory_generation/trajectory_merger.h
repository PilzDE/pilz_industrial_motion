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

#ifndef TRAJECTORY_MERGER_H
#define TRAJECTORY_MERGER_H

#include <moveit/robot_trajectory/robot_trajectory.h>

namespace pilz_trajectory_generation
{

/**
 * @brief Base class for merging two trajectories.
 */
class TrajectoryMerger
{
public:
    /**
     * @brief Merge trajectory into current result trajectory.
     */
    virtual void merge(robot_trajectory::RobotTrajectory &result, const robot_trajectory::RobotTrajectory &source) = 0;
};

}  // namespace pilz_trajectory_generation

#endif  // TRAJECTORY_MERGER_H
