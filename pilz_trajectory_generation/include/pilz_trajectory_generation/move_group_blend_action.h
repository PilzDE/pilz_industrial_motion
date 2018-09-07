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

#ifndef BLEND_ACTION_CAPABILITY_H
#define BLEND_ACTION_CAPABILITY_H

#include <memory>

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>

#include <pilz_msgs/MoveGroupBlendAction.h>

namespace pilz_trajectory_generation
{

class CommandListManager;

/**
 * @brief Provide action to blend multiple trajectories and execute the result
 * in the form of a MoveGroup capability (plugin).
 */
class MoveGroupBlendAction : public move_group::MoveGroupCapability
{
public:
  MoveGroupBlendAction();

  virtual void initialize() override;

private:
  void executeBlendCallback(const pilz_msgs::MoveGroupBlendGoalConstPtr &goal);
  void executeBlendCallback_PlanAndExecute(const pilz_msgs::MoveGroupBlendGoalConstPtr& goal,
                                          pilz_msgs::MoveGroupBlendResult& action_res);
  void executeMoveCallback_PlanOnly(const pilz_msgs::MoveGroupBlendGoalConstPtr& goal,
                                    pilz_msgs::MoveGroupBlendResult& action_res);
  void startMoveExecutionCallback();
  void startMoveLookCallback();
  void preemptMoveCallback();
  void setMoveState(move_group::MoveGroupState state);
  bool planUsingBlendManager(const pilz_msgs::MotionBlendRequestList &req,
                                 plan_execution::ExecutableMotionPlan& plan);
private:
  std::unique_ptr<actionlib::SimpleActionServer<pilz_msgs::MoveGroupBlendAction> > move_action_server_;
  pilz_msgs::MoveGroupBlendFeedback move_feedback_;

  move_group::MoveGroupState move_state_ {move_group::IDLE};
  std::unique_ptr<pilz_trajectory_generation::CommandListManager> blend_manager_;
};
}

#endif // BLEND_ACTION_CAPABILITY_H
