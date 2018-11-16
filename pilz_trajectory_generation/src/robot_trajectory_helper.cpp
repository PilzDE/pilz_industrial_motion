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

#include <pilz_trajectory_generation/robot_trajectory_helper.h>
#include <pilz_trajectory_generation/trajectory_functions.h>
#include <ros/console.h>

namespace pilz_trajectory_generation
{

/**
 * @brief Appends a trajectory to another trajectory leaving out the first state, if it matches the last point of the
 * first trajectory.
 *
 * @note Controllers, so far at least the ros_controllers::JointTrajectoryController require a timewise strictly
 * increasing trajectory. If through appending the last point of the original trajectory gets repeated it is removed here.
 */
void RobotTrajectoryHelper::appendWithoutRedundantFirstState(robot_trajectory::RobotTrajectoryPtr &robot_trajectory,
                                                             robot_trajectory::RobotTrajectory &tail)
{
  size_t start_index = 0;
  if ( (robot_trajectory->getWayPointCount() > 0) && pilz::isRobotStateEqual(robot_trajectory->getLastWayPointPtr(),
                                                                             tail.getFirstWayPointPtr(),
                                                                             robot_trajectory->getGroupName(),
                                                                             ROBOT_STATE_EQUALITY_EPSILON) )
  {
    ++start_index;
  }

  for (size_t i = start_index; i < tail.getWayPointCount(); ++i)
  {
    robot_trajectory->addSuffixWayPoint(tail.getWayPoint(i), tail.getWayPointDurationFromPrevious(i));
  }
}

}  // namespace pilz_trajectory_generation