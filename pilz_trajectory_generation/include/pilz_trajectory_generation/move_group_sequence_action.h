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

#ifndef SEQUENCE_ACTION_CAPABILITY_H
#define SEQUENCE_ACTION_CAPABILITY_H

#include <memory>

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>

#include <pilz_msgs/MoveGroupSequenceAction.h>

namespace pilz_trajectory_generation
{

class CommandListManager;

/**
 * @brief MoveGroup capability providing an action server to plan and execute sequences.
 */
class MoveGroupSequenceAction : public move_group::MoveGroupCapability
{
public:
  MoveGroupSequenceAction();

  virtual void initialize() override;

private:
  /**
   * @brief  Directly called if the action server receives a new goal
   *
   * Redirects based on the `goal->planning_options.plan_only` flag to the subsequent
   * MoveGroupSequenceAction::executeSequenceCallback_PlanAndExecute or MoveGroupSequenceAction::executeMoveCallback_PlanOnly callback.
   */
  void executeSequenceCallback(const pilz_msgs::MoveGroupSequenceGoalConstPtr &goal);
  void executeSequenceCallback_PlanAndExecute(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                          pilz_msgs::MoveGroupSequenceResult& action_res);
  void executeMoveCallback_PlanOnly(const pilz_msgs::MoveGroupSequenceGoalConstPtr& goal,
                                    pilz_msgs::MoveGroupSequenceResult& action_res);

  /** @brief Passed via plan_execution::PlanExecution::Options to plan_execution::PlanExecution.
   *
   *  Used to changes the state to move_group::MONITOR
   */
  void startMoveExecutionCallback();

  /// Invoked when a new preempt request is available. Stops the plan execution.
  void preemptMoveCallback();

  /// Called if the internal state changes. See http://docs.ros.org/kinetic/api/moveit_ros_move_group/html/namespacemove__group.html#a4a295c26dbc5ac7780dfa7ae10350bfb
  void setMoveState(move_group::MoveGroupState state);

  /// Passed via plan_execution::PlanExecution::Options to plan_execution::PlanExecution
  bool planUsingSequenceManager(const pilz_msgs::MotionSequenceRequest &req,
                                 plan_execution::ExecutableMotionPlan& plan);
private:
  std::unique_ptr<actionlib::SimpleActionServer<pilz_msgs::MoveGroupSequenceAction> > move_action_server_;
  pilz_msgs::MoveGroupSequenceFeedback move_feedback_;

  move_group::MoveGroupState move_state_ {move_group::IDLE};
  std::unique_ptr<pilz_trajectory_generation::CommandListManager> sequence_manager_;
};
}

#endif // SEQUENCE_ACTION_CAPABILITY_H
