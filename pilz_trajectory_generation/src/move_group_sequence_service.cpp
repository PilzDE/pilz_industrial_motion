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

#include "pilz_trajectory_generation/move_group_sequence_service.h"

#include "pilz_trajectory_generation/capability_names.h"
#include "pilz_trajectory_generation/command_list_manager.h"

namespace pilz_trajectory_generation
{

MoveGroupSequenceService::MoveGroupSequenceService() : MoveGroupCapability("SequenceService")
{
}

MoveGroupSequenceService::~MoveGroupSequenceService()
{
}

void MoveGroupSequenceService::initialize()
{
  sequence_manager_.reset(new pilz_trajectory_generation::CommandListManager(ros::NodeHandle("~"),
                                                                          context_->planning_scene_monitor_->getRobotModel()));

  sequence_service_ = root_node_handle_.advertiseService(SEQUENCE_SERVICE_NAME,
                                                         &MoveGroupSequenceService::plan,
                                                         this);
}




bool MoveGroupSequenceService::plan(pilz_msgs::GetMotionSequence::Request& req,
                                 pilz_msgs::GetMotionSequence::Response& res)
{
  // TODO: Do we lock on the correct scene? Does the lock belong to the scene used for planning?
  planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);

  // If 'FALSE' then no response will be sent to the caller.
  bool sentResponseToCaller  {true};
  try
  {
    planning_interface::MotionPlanResponse mp_res;
    sequence_manager_->solve(ps, req.commands, mp_res);
    mp_res.getMessage(res.plan_response);
  }
  // LCOV_EXCL_START // Keep moveit up even if lower parts throw
  catch (...)
  {
    ROS_ERROR("Planner threw an exception.");
    sentResponseToCaller = false;
  }
  // LCOV_EXCL_STOP

  return sentResponseToCaller;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pilz_trajectory_generation::MoveGroupSequenceService, move_group::MoveGroupCapability)
