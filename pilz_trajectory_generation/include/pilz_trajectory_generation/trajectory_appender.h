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

#ifndef TRAJECTORY_APPENDER_H
#define TRAJECTORY_APPENDER_H

#include <pilz_trajectory_generation/trajectory_merger.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace pilz_trajectory_generation
{

class TrajectoryAppender : public TrajectoryMerger
{
public:
    /**
     * @brief Appends a trajectory to a result trajectory leaving out the first point, if it matches the last point of
     * the result trajectory.
     *
     * @note Controllers, so far at least the ros_controllers::JointTrajectoryController require a timewise strictly
     * increasing trajectory. If through appending the last point of the original trajectory gets repeated it is removed
     * here.
     */
    void merge(robot_trajectory::RobotTrajectory &result, const robot_trajectory::RobotTrajectory &source) override;

    //! Constant to check for equality of variables of two RobotState instances.
    static constexpr double ROBOT_STATE_EQUALITY_EPSILON = 1e-4;
};

}  // namespace pilz_trajectory_generation

#endif  // TRAJECTORY_APPENDER_H
