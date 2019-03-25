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

#include "pilz_trajectory_generation/command_list_manager.h"

#include <sstream>
#include <functional>
#include <cassert>

#include <ros/ros.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "pilz_trajectory_generation/cartesian_limits_aggregator.h"
#include "pilz_trajectory_generation/trajectory_blender_transition_window.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"
#include "pilz_trajectory_generation/tip_frame_getter.h"

namespace pilz_trajectory_generation
{

static const std::string PARAM_NAMESPACE_LIMTS = "robot_description_planning";

CommandListManager::CommandListManager(const ros::NodeHandle &nh, const moveit::core::RobotModelConstPtr &model):
  nh_(nh),
  model_(model)
{
  // Obtain the aggregated joint limits
  pilz::JointLimitsContainer aggregated_limit_active_joints;

  aggregated_limit_active_joints = pilz::JointLimitsAggregator::getAggregatedLimits(
        ros::NodeHandle(PARAM_NAMESPACE_LIMTS),model_->getActiveJointModels());


  // Obtain cartesian limits
  pilz::CartesianLimit cartesian_limit = pilz::CartesianLimitsAggregator::getAggregatedLimits(ros::NodeHandle(PARAM_NAMESPACE_LIMTS));

  pilz::LimitsContainer limits;
  limits.setJointLimits(aggregated_limit_active_joints);
  limits.setCartesianLimits(cartesian_limit);

  plan_comp_builder_.setModel(model);
  plan_comp_builder_.setBlender(std::unique_ptr<pilz::TrajectoryBlender>(new pilz::TrajectoryBlenderTransitionWindow(limits)));
}

RobotTrajVec_t CommandListManager::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                         const pilz_msgs::MotionSequenceRequest& req_list)
{
  if(req_list.items.empty()) { return RobotTrajVec_t(); }

  checkForNegativeRadii(req_list);
  checkLastBlendRadius(req_list);
  checkForEndEffectorBlending(req_list);
  checkStartStates(req_list);

  MotionResponseCont resp_cont {solveSequenceItems(planning_scene, req_list)};

  RadiiCont radii {getRadii(req_list)};
  checkForOverlappingRadii(resp_cont, radii);

  plan_comp_builder_.reset();
  for(MotionResponseCont::size_type i = 0; i < resp_cont.size(); ++i)
  {
    plan_comp_builder_.append(resp_cont.at(i).trajectory_,
                              // The blend radii has to be "attached" to
                              // the second part of a blend trajectory,
                              // therefore: "i-1".
                              ( i>0? radii.at(i-1) : 0.) );
  }
  return plan_comp_builder_.build();
}

bool CommandListManager::checkRadiiForOverlap(const robot_trajectory::RobotTrajectory& traj_A,
                                              const double radii_A,
                                              const robot_trajectory::RobotTrajectory& traj_B,
                                              const double radii_B) const
{
  // No blending between trajectories from different groups.
  if (traj_A.getGroupName() != traj_B.getGroupName()) {return false;}

  auto sum_radii {radii_A + radii_B};
  if(sum_radii == 0.) {return false;}

  const std::string& blend_frame {getTipFrame(model_->getJointModelGroup(traj_A.getGroupName()))};
  auto distance_endpoints = (traj_A.getLastWayPoint().getFrameTransform(blend_frame).translation() -
                             traj_B.getLastWayPoint().getFrameTransform(blend_frame).translation()).norm();
  return distance_endpoints <= sum_radii;
}

void CommandListManager::checkForOverlappingRadii(const MotionResponseCont &resp_cont,
                                                  const RadiiCont &radii) const
{
  if(resp_cont.empty()) { return; }
  if(resp_cont.size() < 3) { return; }

  for(MotionResponseCont::size_type i = 0; i < resp_cont.size()-2; ++i)
  {
    if (checkRadiiForOverlap(*(resp_cont.at(i).trajectory_), radii.at(i),
                             *(resp_cont.at(i+1).trajectory_), radii.at(i+1)))
    {
      std::ostringstream os;
      os << "Overlapping blend radii between command [" << i << "] and [" << i+1 << "].";
      throw OverlappingBlendRadiiException(os.str());
    }
  }
}

CommandListManager::RobotState_OptRef  CommandListManager::getPreviousEndState(const MotionResponseCont &motion_plan_responses,
                                                                               const std::string& group_name)
{
  for(MotionResponseCont::const_reverse_iterator it = motion_plan_responses.crbegin(); it != motion_plan_responses.crend(); ++it)
  {
    if (it->trajectory_->getGroupName() == group_name)
    {
      return it->trajectory_->getLastWayPoint();
    }
  }
  return boost::none;
}

void CommandListManager::setStartState(const MotionResponseCont &motion_plan_responses,
                                       const std::string &group_name,
                                       moveit_msgs::RobotState& start_state)
{
  RobotState_OptRef robStateOpt {getPreviousEndState(motion_plan_responses, group_name)};
  if (robStateOpt)
  {
    moveit::core::robotStateToRobotStateMsg(robStateOpt.value(), start_state);
  }
}

CommandListManager::RadiiCont CommandListManager::getRadii(const pilz_msgs::MotionSequenceRequest &req_list)
{
  RadiiCont radii(req_list.items.size(), 0.);
  RadiiCont::size_type i {0};
  for(const auto& req : req_list.items)
  {
    radii.at(i) = req.blend_radius;
    ++i;
  }
  return radii;
}

CommandListManager::MotionResponseCont CommandListManager::solveSequenceItems(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const pilz_msgs::MotionSequenceRequest &req_list) const
{
  MotionResponseCont motion_plan_responses;
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(model_, nh_));
  size_t curr_req_index {0};
  const size_t num_req {req_list.items.size()};
  for(const auto& seq_item : req_list.items)
  {
    planning_interface::MotionPlanRequest req {seq_item.req};
    setStartState(motion_plan_responses, req.group_name, req.start_state);

    planning_interface::MotionPlanResponse res;
    planning_pipeline->generatePlan(planning_scene, req, res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      std::ostringstream os;
      os << "Could not solve request\n---\n" << req << "\n---\n";
      throw PlanningPipelineException(os.str(), res.error_code_.val);
    }
    motion_plan_responses.emplace_back(res);
    ROS_DEBUG_STREAM("Solved [" << ++curr_req_index << "/" << num_req << "]");
  }
  return motion_plan_responses;
}

void CommandListManager::checkForEndEffectorBlending(const pilz_msgs::MotionSequenceRequest &req_list)
{
  for(pilz_msgs::MotionSequenceRequest::_items_type::size_type i = 0; i < req_list.items.size()-1; ++i)
  {
    const pilz_msgs::MotionSequenceItem& item_A {req_list.items.at(i)};
    if (item_A.blend_radius <= 0)
    {
      continue;
    }
    const pilz_msgs::MotionSequenceItem& item_B {req_list.items.at(i+1)};
    if (item_A.req.group_name != item_B.req.group_name)
    {
      continue;
    }
    if (model_->getJointModelGroup(item_A.req.group_name)->isEndEffector())
    {
      std::ostringstream os;
      os << "Blending of two End-Effector trajectories between [" << i << "] and [" << i+1 << "]";
      throw EndEffectorBlendingException(os.str());
    }
  }
}

void CommandListManager::checkForNegativeRadii(const pilz_msgs::MotionSequenceRequest &req_list)
{
  if(!std::all_of(req_list.items.begin(), req_list.items.end(),
                  [](const pilz_msgs::MotionSequenceItem& req){return (req.blend_radius >= 0.);}))
  {
    throw NegativeBlendRadiusException("All blending radii MUST be non negative");
  }
}

void CommandListManager::checkStartStates(const pilz_msgs::MotionSequenceRequest &req_list)
{
  if (req_list.items.size() <= 1)
  {
    return;
  }

  if(std::any_of(req_list.items.begin()+1, req_list.items.end(),
                 [](const pilz_msgs::MotionSequenceItem& req){
                 return !(req.req.start_state.joint_state.position.empty()
                          && req.req.start_state.joint_state.velocity.empty()
                          && req.req.start_state.joint_state.effort.empty()
                          && req.req.start_state.joint_state.name.empty());
}))
  {
    throw StartStateSetException("Only the first request is allowed to have a start state");
  }
}

}
