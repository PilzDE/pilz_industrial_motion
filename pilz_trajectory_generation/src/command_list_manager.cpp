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

#include <ros/ros.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

#include "pilz_trajectory_generation/joint_limits_aggregator.h"
#include "pilz_trajectory_generation/cartesian_limits_aggregator.h"
#include "pilz_trajectory_generation/trajectory_blender_transition_window.h"
#include "pilz_trajectory_generation/trajectory_blend_request.h"

namespace pilz_trajectory_generation {

static const std::string PARAM_NAMESPACE_LIMTS = "robot_description_planning";
static const double point_identity_threshold=10e-5;

//CTOR
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

  // Currently using Lloyed blender
  std::unique_ptr<pilz::TrajectoryBlender> blender(new pilz::TrajectoryBlenderTransitionWindow(limits));
  blender_ = std::move(blender);
}

bool CommandListManager::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const pilz_msgs::MotionSequenceRequest &req_list,
                               planning_interface::MotionPlanResponse& res)
{
  //*****************************
  // Validations
  //*****************************

  // Return true on empty request
  if(req_list.items.empty())
  {
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
  }

  if(!validateRequestList(req_list, res))
  {
    return false;
  }

  //*****************************
  // Solve all requests
  //*****************************

  // Collect the responses
  std::vector<planning_interface::MotionPlanResponse> motion_plan_responses;
  std::vector<double> radii;

  if(!solveRequests(planning_scene, req_list, res, motion_plan_responses, radii))
  {
    return false;
  }

  //*****************************
  // Validate that the blending radii do not overlap
  //*****************************

  const auto group_name = req_list.items.front().req.group_name;

  if(!validateBlendingRadiiDoNotOverlap(motion_plan_responses, radii, group_name))
  {
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  //*****************************
  // Blending
  //*****************************

  // Prepare the result trajectory
  robot_trajectory::RobotTrajectoryPtr result_trajectory(
        new robot_trajectory::RobotTrajectory(model_, motion_plan_responses.front().trajectory_->getGroupName()));


  // Case: Only one trajectory in request
  if(req_list.items.size() == 1)
  {
    res.trajectory_ = motion_plan_responses[0].trajectory_;
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    return true;
  }

  if(!generateTrajectory(motion_plan_responses, radii, result_trajectory, res))
  {
    return false;
  }

  //*****************************
  // Create the response
  //*****************************

  res.trajectory_ = result_trajectory;
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}

bool CommandListManager::validateRequestList(const pilz_msgs::MotionSequenceRequest &req_list,
                                             planning_interface::MotionPlanResponse &res)
{
  // Check that all request are about the same group
  // const auto group_name = req_list.items.front().req.group_name;
  // if(! std::all_of(req_list.items.begin(), req_list.items.end(),
  //                  [group_name](const pilz_msgs::MotionSequenceItem& req){
  //                     if(req.req.group_name != group_name) {
  //                       ROS_ERROR_STREAM("Cannot blend. All requests MUST be about the same group!"
  //                                        << " Groups: '" << req.req.group_name << "' and '" << group_name << "'"
  //                                        << " can not be used in the same sequence");
  //                       return false;
  //                     }

  //                     return true;
  //                   }))
  // {
  //   ROS_ERROR_STREAM("Cannot blend. All requests MUST be about the same group!");
  //   res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
  //   res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
  //   return false;
  // }

  // Validate that all blending radius are non-negative
  if(! std::all_of(req_list.items.begin(), req_list.items.end(),
                   [](const pilz_msgs::MotionSequenceItem& req){return (req.blend_radius >= 0.0);}))
  {
    ROS_ERROR_STREAM("Cannot blend. All blending radii MUST be non negative!");
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Validate that the last blending radius is 0
  if(req_list.items.back().blend_radius != 0.0)
  {
    ROS_ERROR_STREAM("Cannot blend. The last blending radius must be zero!");
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  // Only the first request is allowed to have a start_state. The others are checked here for empty start_state.
  if(req_list.items.size() > 1 &&
     std::any_of(req_list.items.begin()+1, req_list.items.end(),
        [](const pilz_msgs::MotionSequenceItem& req){
           return !(req.req.start_state.joint_state.position.empty()
                   && req.req.start_state.joint_state.velocity.empty()
                   && req.req.start_state.joint_state.effort.empty()
                   && req.req.start_state.joint_state.name.empty());
        }))
  {
    ROS_ERROR_STREAM("Cannot blend. Only the first request is allowed to have a start state!");
    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
    return false;
  }

  return true;
}

bool CommandListManager::validateBlendingRadiiDoNotOverlap(
    const std::vector<planning_interface::MotionPlanResponse> &motion_plan_responses,
    const std::vector<double> &radii,
    const std::string& group_name)
{
  if(motion_plan_responses.size() > 1)
  {
    for(unsigned long i = 0; i < motion_plan_responses.size()-2; i++)
    {
      auto sum_radii = radii.at(i) + radii.at(i+1);

      if(sum_radii == 0) continue;

      auto traj_1 = motion_plan_responses.at(i).trajectory_;
      auto traj_2 = motion_plan_responses.at(i+1).trajectory_;
      auto distance_endpoints = (traj_1->getLastWayPoint().getFrameTransform(getTipFrame(group_name)).translation() -
                                 traj_2->getLastWayPoint().getFrameTransform(getTipFrame(group_name)).translation())
                                .norm();

      if(distance_endpoints <= (radii.at(i) + radii.at(i+1)))
      {
        ROS_ERROR_STREAM("Overlapping blend radii between command [" << i << "] and [" << i+1 << "].");
        return false;
      }
    }
  }

  return true;
}

bool CommandListManager::solveRequests(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const pilz_msgs::MotionSequenceRequest &req_list,
                                       planning_interface::MotionPlanResponse &res,
                                       std::vector<planning_interface::MotionPlanResponse> &motion_plan_responses,
                                       std::vector<double> &radii)
{
  // Obtain the planning pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(model_, nh_));

  for(auto req_it = req_list.items.begin(); req_it < req_list.items.end(); req_it++)
  {
    size_t idx = std::distance(req_list.items.begin(), req_it);

    planning_interface::MotionPlanRequest req = req_it->req;
    planning_interface::MotionPlanResponse plan_res;

    // Set start state of request to end state of previous trajectory (except for first)
    if(req_it != req_list.items.begin())
    {
      moveit::core::robotStateToRobotStateMsg(motion_plan_responses.back().trajectory_->getLastWayPoint(),
                                              req.start_state);
    }

    planning_pipeline->generatePlan(planning_scene, req, plan_res);
    /* Check that the planning was successful */
    if (plan_res.error_code_.val != plan_res.error_code_.SUCCESS)
    {
      ROS_DEBUG_STREAM("Could not solve request \n ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n"
                       << req << "\n" << idx << " error_code " << plan_res.error_code_.val << "\n~~~~~~~~~~~~~~~~~~~~");
      res = plan_res;
      res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0)); // This should be done in the planning plugin already
      return false;
    }

    ROS_DEBUG_STREAM("Solved [" << idx+1 << "/" << req_list.items.size() << "]");

    motion_plan_responses.push_back(plan_res);
    radii.push_back(req_it->blend_radius);
  }

  return true;
}

bool CommandListManager::generateTrajectory(
                               const std::vector<planning_interface::MotionPlanResponse> &motion_plan_responses,
                               const std::vector<double> &radii,
                               robot_trajectory::RobotTrajectoryPtr& result_trajectory,
                               planning_interface::MotionPlanResponse &res)
{
  // prefill the first_trajectory for the next blending request
  auto first_trajectory = motion_plan_responses.front().trajectory_;

  for(size_t i = 0; i < motion_plan_responses.size()-1; i++)
  {
    auto traj_2 = motion_plan_responses.at(i+1).trajectory_;
    auto blend_radius = radii.at(i);

    // No blending is needed if the radius is 0.0
    if(blend_radius > 0.0)
    {
      // Generate Blend Request
      pilz::TrajectoryBlendRequest blend_request;

      // The blending is always done between the rest of the previous segment and the new part
      blend_request.first_trajectory = first_trajectory;

      blend_request.second_trajectory = traj_2;
      blend_request.blend_radius = blend_radius;
      blend_request.group_name = first_trajectory->getGroupName();
      blend_request.link_name = getTipFrame(blend_request.group_name);

      // The response
      pilz::TrajectoryBlendResponse blend_response;
      if (!blender_->blend(blend_request, blend_response))
      {
        ROS_ERROR("Blending failed.");
        res.trajectory_.reset(new robot_trajectory::RobotTrajectory(model_, 0));
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
      }

      // Append the new trajectory
      result_trajectory->append(*blend_response.first_trajectory, 0.0);
      result_trajectory->append(*blend_response.blend_trajectory, 0.0);
      first_trajectory = blend_response.second_trajectory; // first for next blending segment
    }
    // if blend radius == 0.0
    else
    {
      appender_.merge(*result_trajectory, *first_trajectory);
      first_trajectory = traj_2;
    }
  }

  appender_.merge(*result_trajectory, *first_trajectory); // append tail
  return true;
}

const std::string &CommandListManager::getTipFrame(const std::string& group_name)
{
  auto group = model_->getJointModelGroup(group_name);
  if(group->isEndEffector()) {
    auto links = group->getLinkModels();

    if(links.size() > 1) {
      throw std::runtime_error("Endeffector with multiple links is not supported");
    }

    if(links.size() < 1) {
      throw std::runtime_error("Endeffector with no links is not supported");
    }

    return links.front()->getName();
  }

  auto solver = group->getSolverInstance();

  if(solver == nullptr)
  {
    throw std::runtime_error("No solver for group " + group_name);
  }

  return solver->getTipFrame();
}

}
