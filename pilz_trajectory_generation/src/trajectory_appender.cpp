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

#include <pilz_trajectory_generation/trajectory_appender.h>
#include <pilz_trajectory_generation/trajectory_functions.h>

namespace pilz_trajectory_generation
{

void TrajectoryAppender::merge(robot_trajectory::RobotTrajectory &result, const robot_trajectory::RobotTrajectory &source)
{
  if ( !result.empty() && pilz::isRobotStateEqual(result.getLastWayPoint(), source.getFirstWayPoint(),
                                                  result.getGroupName(), ROBOT_STATE_EQUALITY_EPSILON) )
  {
    for (size_t i = 1; i < source.getWayPointCount(); ++i)
    {
      result.addSuffixWayPoint(source.getWayPoint(i), source.getWayPointDurationFromPrevious(i));
    }
  }
  else
  {
    result.append(source, 0.0);
  }
}

}  // namespace pilz_trajectory_generation
