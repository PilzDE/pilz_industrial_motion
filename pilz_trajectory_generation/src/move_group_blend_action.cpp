/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

// Modified by Pilz GmbH & Co. KG

#include "pilz_trajectory_generation/move_group_blend_action.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>

#include "pilz_trajectory_generation/command_list_manager.h"

namespace pilz_trajectory_generation
{

MoveGroupBlendAction::MoveGroupBlendAction()
  : MoveGroupCapability("BlendAction")
{
}

void MoveGroupBlendAction::initialize()
{
  // start the move action server
  move_action_server_.reset( new actionlib::SimpleActionServer<pilz_msgs::MoveGroupBlendAction>(
                               root_node_handle_, "blend_move_group",
                               boost::bind(&MoveGroupBlendAction::executeBlendCallback, this, _1), false) );
  move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupBlendAction::preemptMoveCallback, this));
  move_action_server_->start();

  blend_manager_.reset(new pilz_trajectory_generation::CommandListManager (
                         ros::NodeHandle("~"), context_->planning_scene_monitor_->getRobotModel()));

}

void MoveGroupBlendAction::executeBlendCallback(const pilz_msgs::MoveGroupBlendGoalConstPtr& goal)
{
  setMoveState(move_group::PLANNING);
  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  pilz_msgs::MoveGroupBlendResult action_res;
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
    {
      ROS_WARN("Only plan will be calculated, although plan_only == false.");
    }
    executeMoveCallback_PlanOnly(goal, action_res);
  }
  else
  {
    executeBlendCallback_PlanAndExecute(goal, action_res);
  }

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
  std::string response =
      getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);

  switch(action_res.error_code.val)
  {
  case moveit_msgs::MoveItErrorCodes::SUCCESS:
    move_action_server_->setSucceeded(action_res, response);
    break;
  case moveit_msgs::MoveItErrorCodes::PREEMPTED:
    move_action_server_->setPreempted(action_res, response);
    break;
  default:
    move_action_server_->setAborted(action_res, response);
    break;
  }

  setMoveState(move_group::IDLE);
}

void MoveGroupBlendAction::executeBlendCallback_PlanAndExecute(const pilz_msgs::MoveGroupBlendGoalConstPtr& goal,
                                                               pilz_msgs::MoveGroupBlendResult& action_res)
{
  ROS_INFO("Combined planning and execution request received for MoveGroupBlendAction.");

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::PlanningScene& planning_scene_diff =
      planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
        goal->planning_options.planning_scene_diff :
        clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupBlendAction::startMoveExecutionCallback, this);

  opt.plan_callback_ =
      boost::bind(&MoveGroupBlendAction::planUsingBlendManager, this, boost::cref(goal->request), _1);

  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    //TODO: test the following code block with sensing
//    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(),
//                                     _1, opt.plan_callback_, goal->planning_options.look_around_attempts,
//                                     goal->planning_options.max_safe_execution_cost);
//    context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupBlendAction::startMoveLookCallback, this));

    ROS_WARN("Plan with sensing not yet implemented/tested. This option is ignored.");
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
  {
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  }
  action_res.error_code = plan.error_code_;
}

void MoveGroupBlendAction::executeMoveCallback_PlanOnly(const pilz_msgs::MoveGroupBlendGoalConstPtr& goal,
                                                        pilz_msgs::MoveGroupBlendResult& action_res)
{
  ROS_INFO("Planning request received for MoveGroupBlendAction action.");

  // lock the scene so that it does not modify the world representation while diff() is called
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  const planning_scene::PlanningSceneConstPtr& the_scene =
      (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
        static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
        lscene->diff(goal->planning_options.planning_scene_diff);

  planning_interface::MotionPlanResponse res;
  try
  {
    blend_manager_->solve(the_scene, goal->request, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

bool MoveGroupBlendAction::planUsingBlendManager(const pilz_msgs::MotionBlendRequestList& req,
                                                     plan_execution::ExecutableMotionPlan& plan)
{
  setMoveState(move_group::PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  bool solved = false;
  planning_interface::MotionPlanResponse res;
  try
  {
    solved = blend_manager_->solve(plan.planning_scene_, req, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void MoveGroupBlendAction::startMoveExecutionCallback()
{
  setMoveState(move_group::MONITOR);
}

void MoveGroupBlendAction::startMoveLookCallback()
{
  setMoveState(move_group::LOOK);
}

void MoveGroupBlendAction::preemptMoveCallback()
{
  context_->plan_execution_->stop();
}

void MoveGroupBlendAction::setMoveState(move_group::MoveGroupState state)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  move_action_server_->publishFeedback(move_feedback_);
}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pilz_trajectory_generation::MoveGroupBlendAction, move_group::MoveGroupCapability)

