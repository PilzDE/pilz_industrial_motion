/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PLANCOMPONENTSBUILDER_H
#define PLANCOMPONENTSBUILDER_H

#include <string>
#include <functional>
#include <memory>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "pilz_trajectory_generation/trajectory_functions.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/trajectory_blender.h"
#include "pilz_trajectory_generation/trajectory_generation_exceptions.h"

namespace pilz_trajectory_generation
{

using TipFrameFunc_t = std::function<const std::string&(const std::string&)>;

CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoBlenderSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoTipFrameFunctionSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(NoRobolModelSetException, moveit_msgs::MoveItErrorCodes::FAILURE);
CREATE_MOVEIT_ERROR_CODE_EXCEPTION(BlendingFailedException, moveit_msgs::MoveItErrorCodes::FAILURE);

class PlanComponentsBuilder
{
public:
  void setBlender(std::unique_ptr<pilz::TrajectoryBlender> blender);
  void setModel(const moveit::core::RobotModelConstPtr &model);
  void setTipFrameFunc(TipFrameFunc_t func);

  void append(robot_trajectory::RobotTrajectoryPtr other, const double blend_radius);
  void reset();

  std::vector<robot_trajectory::RobotTrajectoryPtr> build() const;

private:
  void blend(robot_trajectory::RobotTrajectoryPtr other,
             const double blend_radius);

private:
  /**
   * @brief Appends a trajectory to a result trajectory leaving out the
   * first point, if it matches the last point of the result trajectory.
   *
   * @note Controllers, so far at least the
   * ros_controllers::JointTrajectoryController require a timewise strictly
   * increasing trajectory. If through appending the last point of the
   * original trajectory gets repeated it is removed here.
   */
  static void appendWithStrictTimeIncrease(robot_trajectory::RobotTrajectory &result,
                                           const robot_trajectory::RobotTrajectory &source);

private:
  std::unique_ptr<pilz::TrajectoryBlender> blender_;
  moveit::core::RobotModelConstPtr model_;
  TipFrameFunc_t tipFrameFunc_;

  robot_trajectory::RobotTrajectoryPtr traj_tail_;
  std::vector<robot_trajectory::RobotTrajectoryPtr> traj_cont_;

private:
  //! Constant to check for equality of variables of two RobotState instances.
  static constexpr double ROBOT_STATE_EQUALITY_EPSILON = 1e-4;
};

inline void PlanComponentsBuilder::setBlender(std::unique_ptr<pilz::TrajectoryBlender> blender)
{
  blender_ = std::move(blender);
}

inline void PlanComponentsBuilder::setTipFrameFunc(TipFrameFunc_t func)
{
  tipFrameFunc_ = func;
}

inline void PlanComponentsBuilder::setModel(const moveit::core::RobotModelConstPtr &model)
{
  model_ = model;
}

inline void PlanComponentsBuilder::reset()
{
  traj_tail_ = nullptr;
  traj_cont_.clear();
}


}

#endif // PLANCOMPONENTSBUILDER_H
